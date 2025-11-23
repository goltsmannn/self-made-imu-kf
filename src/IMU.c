#include "IMU.h"

const uint8_t accel_setup_commands[4] = {
    CTRL_REG5_XL, 0b00111000,
    CTRL_REG6_XL, 0b01000000
};

void imu_init() {
    uint8_t raw_data[8]; // Assuming 2 commands, each with 4 bytes response
    imu_multi_read_task_data_t accel_setup_task = {
        .commands = accel_setup_commands,
        .command_length = 2,
        .num_commands = 2,
        .dummy_length = 0,
        .device_type = ACCELEROMETER,
        .responses = raw_data,
        .read_active_high = 0
    };

    imu_send_multiple_commands(&accel_setup_task);
}

void package_data(uint8_t *raw_data, sensor_data* sensor) {
    sensor->x = (int16_t)((raw_data[1] << 8) | raw_data[5]);
    sensor->y = (int16_t)((raw_data[9] << 8) | raw_data[13]);
    sensor->z = (int16_t)((raw_data[17] << 8) | raw_data[21]);
}

void imu_spi_transfer_task(void *pvParameters) {
    uint8_t rx[4];
    spi_task_data_t *task_data = (spi_task_data_t *)pvParameters;
    task_data->response = rx;


    while (1) {
        imu_spi_transfer(task_data);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000 milliseconds
        printf("IMU SPI transfer task running...\n");
        for (int i = 0; i < task_data->command_length + task_data->dummy_length; i++) {
            printf("Byte %d: %d\n", i, task_data->response[i]);
        }
    }
}

void imu_send_multiple_commands(imu_multi_read_task_data_t *task_data) {
    uint8_t *raw_data = task_data->responses;

    spi_task_data_t spi_single_task = {
        .command_length = task_data->command_length,
        .dummy_length = task_data->dummy_length,
        .device_type = task_data->device_type,
        .read_active_high = task_data->read_active_high,
    };

    for (int i = 0; i < task_data->num_commands; ++i) {
        spi_single_task.command = &(task_data->commands[i * task_data->command_length]);
        spi_single_task.response = raw_data + (i * 4);
        imu_spi_transfer(&spi_single_task);
    }
}

void read_mult_task(void *pvParameters) {
    // uint8_t commands[6] = {OUT_X_H_XL, OUT_X_L_XL, OUT_Y_H_XL, OUT_Y_L_XL, OUT_Z_H_XL, OUT_Z_L_XL};
    uint8_t commands[6] = {WHO_AM_I_XG, WHO_AM_I_XG, WHO_AM_I_XG, WHO_AM_I_XG, WHO_AM_I_XG, WHO_AM_I_XG};
    // uint8_t commands[6] = {OUT_X_H_G, OUT_X_L_G, OUT_Y_H_G, OUT_Y_L_G, OUT_Z_H_G, OUT_Z_L_G};
    // uint8_t commands[6] = {OUT_X_H_M, OUT_X_L_M, OUT_Y_H_M, OUT_Y_L_M, OUT_Z_H_M, OUT_Z_L_M};
    uint8_t raw_data[24] = {0};
    imu_multi_read_task_data_t task_data = {
        .commands = commands,
        .responses = raw_data,
        .command_length = 1,
        .device_type = ACCELEROMETER,
        .dummy_length = 1,
        .read_active_high = 1,
        .num_commands = 6
    };

    sensor_data sensor;
    package_data(raw_data, &sensor);
    while (1) {
        imu_send_multiple_commands(&task_data);
        printf("Accel Data - X: %d, Y: %d, Z: %d\n", sensor.x, sensor.y, sensor.z);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void imu_spi_transfer(spi_task_data_t *task_data) {

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

    for (int i = 0; i < task_data->command_length + task_data->dummy_length; i++) {
        printf("TX Byte %d: %d\n", i, tx[i]);
    }


    spi_transmit(tx, task_data->response, device, task_data->command_length + task_data->dummy_length);
}

