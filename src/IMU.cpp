#include "IMU.h"

const uint8_t accel_setup_commands[4] = {
    CTRL_REG5_XL, 0b00111000,
    CTRL_REG6_XL, 0b01000000
};

const uint8_t gyro_setup_commands[6] = {
    CTRL_REG1_G, 0b00100000, // exit powerdown
    CTRL_REG4_G, 0b00111000,  // enable axis
};

bool IMU::multi_read_enabled = false;
// void IMU::set_sensor(DeviceType type, sensor_data sensor) {
//     switch (type) {
//         case ACCELEROMETER:
//             data.accel = sensor;
//             break;
//         case GYROSCOPE:
//             data.gyro = sensor;
//             break;
//         case MAGNETOMETER:
//             data.mag = sensor;
//             break;
//         default:
//             break;
//     }
// }

void IMU::package_data(uint8_t *raw_data, sensor_data* sensor) {
    sensor->x = (int16_t)((raw_data[1] << 8) | raw_data[5]);
    sensor->y = (int16_t)((raw_data[9] << 8) | raw_data[13]);
    sensor->z = (int16_t)((raw_data[17] << 8) | raw_data[21]);
}

void IMU::spi_transmit_single_command_task(void *pvParameters) {
    uint8_t rx[4];
    spi_transmit_single_command_task_s *task_data = (spi_transmit_single_command_task_s *)pvParameters;
    task_data->response = rx;


    while (1) {
        IMU::spi_transmit_single_command(task_data);
        vTaskDelay(pdMS_TO_TICKS(80)); // Delay for 80 milliseconds
        printf("IMU SPI transfer task running...\n");
        for (int i = 0; i < task_data->command_length + task_data->dummy_length; i++) {
            printf("Byte %d: %d\n", i, task_data->response[i]);
        }
    }
}



void IMU::spi_transmit_single_command(spi_transmit_single_command_task_s *task_data) {

    uint8_t tx[4];
    spi_device_handle_t device;

    switch (task_data->device_type) {
        case MAGNETOMETER:
            device = magnetometer_spi_device_handle; 
            tx[0] = (task_data->read_active_high << 7) | (task_data->command[0] & 0x3F);
            break;
        case ACCELEROMETER:
        case GYROSCOPE:
            device = accelerometer_gyroscope_spi_device_handle;
            tx[0] = (task_data->read_active_high << 7) | (task_data->command[0] & 0x7F);
            break;
        default:
            device = NULL;
            break;
    }

    for (int i = 1; i < task_data->command_length; i++) {
        tx[i] = task_data->command[i]; 
    }
    for (int i = 0; i < task_data->dummy_length; i++) {
        tx[task_data->command_length + i] = 0x00; 
    }

    // for (int i = 0; i < task_data->command_length + task_data->dummy_length; i++) {
        // printf("TX Byte %d: %d\n", i, tx[i]);
    // }


    spi_transmit(tx, task_data->response, device, task_data->command_length + task_data->dummy_length);
}

void IMU::init() {
    uint8_t rx_dummy[4];
    spi_transmit_single_command_task_s gyro_init_task = {
        .command = (uint8_t *)gyro_setup_commands,
        .command_length = 2,
        .dummy_length = 0,
        .device_type = GYROSCOPE,
        .response = rx_dummy,
        .read_active_high = 0
    };

    IMU::spi_transmit_single_command(&gyro_init_task); // enable gyro

    gyro_init_task.command = (uint8_t *)&gyro_setup_commands[2];
    IMU::spi_transmit_single_command(&gyro_init_task); // enable axes

    IMU::multi_read_enabled = false;
    IMU::toggle_multi_read();
}

void IMU::read_gyroscope_task(void *pvParameters) {
    while (1) {
        IMU::read_gyroscope();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

uint8_t* IMU::read_gyroscope() {
    if (!multi_read_enabled) {
        IMU::toggle_multi_read();
    }
    static uint8_t gyro_data[24];

    spi_transmit_single_command_task_s get_gyro_data = {
        .command = (uint8_t[]){OUT_X_L_G},
        .command_length = 1,
        .dummy_length = 6,
        .device_type = GYROSCOPE,
        .response = gyro_data,
        .read_active_high = 1
    };
    IMU::spi_transmit_single_command(&get_gyro_data);
    printf("Gyro Data - X: %d, Y: %d, Z: %d\n", 
        gyro_data[0] << 7 | gyro_data[1], gyro_data[2] << 7 | gyro_data[3], gyro_data[4] << 7 | gyro_data[5]);
    return gyro_data;
}

void IMU::toggle_multi_read() {
    uint8_t tx[] = { CTRL_REG8, 0x00 }; // enable multiread
    if (!multi_read_enabled) {
        tx[1] =0b00000100;
    } 

    uint8_t rx_dummy[2];

    spi_transmit_single_command_task_s multi_read = {
        .command = tx,
        .command_length = 2,
        .dummy_length = 0,
        .device_type = GYROSCOPE,
        .response = rx_dummy,
        .read_active_high = 0
    };
    IMU::spi_transmit_single_command(&multi_read);
    multi_read_enabled = !multi_read_enabled;
    if (multi_read_enabled) {
        printf("Multi-read enabled\n");
    } else {
        printf("Multi-read disabled\n");
    }
}

// void read_mult_task(void *pvParameters) {
//     // uint8_t commands[6] = {OUT_X_H_XL, OUT_X_L_XL, OUT_Y_H_XL, OUT_Y_L_XL, OUT_Z_H_XL, OUT_Z_L_XL};
//     uint8_t commands[6] = {WHO_AM_I_XG, WHO_AM_I_XG, WHO_AM_I_XG, WHO_AM_I_XG, WHO_AM_I_XG, WHO_AM_I_XG};
//     // uint8_t commands[6] = {OUT_X_H_G, OUT_X_L_G, OUT_Y_H_G, OUT_Y_L_G, OUT_Z_H_G, OUT_Z_L_G};
//     // uint8_t commands[6] = {OUT_X_H_M, OUT_X_L_M, OUT_Y_H_M, OUT_Y_L_M, OUT_Z_H_M, OUT_Z_L_M};
//     uint8_t raw_data[24] = {0};
//     imu_multi_read_task_data_t task_data = {
//         .commands = commands,
//         .responses = raw_data,
//         .command_length = 1,
//         .device_type = ACCELEROMETER,
//         .dummy_length = 1,
//         .read_active_high = 1,
//         .num_commands = 6
//     };

//     sensor_data sensor;
//     package_data(raw_data, &sensor);
//     while (1) {
//         imu_send_multiple_commands(&task_data);
//         printf("Accel Data - X: %d, Y: %d, Z: %d\n", sensor.x, sensor.y, sensor.z);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

// void imu_send_multiple_commands(imu_multi_read_task_data_t *task_data) {
//     uint8_t *raw_data = task_data->responses;

//     spi_task_data_t spi_single_task = {
//         .command_length = task_data->command_length,
//         .dummy_length = task_data->dummy_length,
//         .device_type = task_data->device_type,
//         .read_active_high = task_data->read_active_high,
//     };

//     for (int i = 0; i < task_data->num_commands; ++i) {
//         spi_single_task.command = &(task_data->commands[i * task_data->command_length]);
//         spi_single_task.response = raw_data + (i * 4);
//         imu_spi_transfer(&spi_single_task);
//     }
// }
