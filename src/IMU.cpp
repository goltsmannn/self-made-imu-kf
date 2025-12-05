#include "IMU.h"

const uint8_t accel_setup_commands[6] = {
    CTRL_REG5_XL, 0b00111000, // enable axis
    CTRL_REG6_XL, 0b01000000, // 50HZ ODR, +-2g, 
    CTRL_REG7_XL, 0b11000100 // ODR/9 BW, High-res mode, use LPF, bypass HPF
    // CTRL_REG7_XL, 0b01000000 // ODR/9 BW, High-res mode, use LPF, bypass HPF
};

const uint8_t gyro_setup_commands[6] = {
    CTRL_REG1_G, 0b01000000, // 59.5hz ODR, slow rotation, 
    CTRL_REG4_G, 0b00111000,  // enable axis
};

const uint8_t mag_setup_commands[4] = {
    CTRL_REG1_M, 0b10011000, // 40HZ rate, temp comp, no high performance, 
    CTRL_REG3_M, 0b10000000 // continuous-conversion mode, i2c disable
};


bool IMU::multi_read_enabled = false;

void IMU::package_data(uint8_t *raw_data, sensor_data* sensor) {
    // this will automatically convert from raw data to readable format
    sensor->x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    sensor->y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    sensor->z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
}

void IMU::convert_raw_data(raw_imu_data* imu_data, converted_imu_data* converted_data, DeviceType device) {
    if (device == ACCELEROMETER) {
        converted_data->accel.x = RAW_TO_ACCEL_G(imu_data->accel.x);
        converted_data->accel.y = RAW_TO_ACCEL_G(imu_data->accel.y);
        converted_data->accel.z = RAW_TO_ACCEL_G(imu_data->accel.z);
    } else if (device == GYROSCOPE) {
        converted_data->gyro.x = RAW_TO_GYRO_DPS(imu_data->gyro.x);
        converted_data->gyro.y = RAW_TO_GYRO_DPS(imu_data->gyro.y);
        converted_data->gyro.z = RAW_TO_GYRO_DPS(imu_data->gyro.z);
    } else if (device == MAGNETOMETER) {    
        converted_data->mag.x = RAW_TO_MAG_GAUSS(imu_data->mag.x);
        converted_data->mag.y = RAW_TO_MAG_GAUSS(imu_data->mag.y);
        converted_data->mag.z = RAW_TO_MAG_GAUSS(imu_data->mag.z);
    }
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
            tx[0] = (task_data->read_active_high << 7) | (task_data->multi_read_active_high << 6) | (task_data->command[0] & 0x3F);
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

    // setup gyro
    spi_transmit_single_command_task_s init_task = {
        .command = (uint8_t *)gyro_setup_commands,
        .command_length = 2,
        .dummy_length = 0,
        .device_type = GYROSCOPE,
        .response = rx_dummy,
        .read_active_high = 0,
        .multi_read_active_high = 1 // doesn't matter for gyro
    };

    IMU::spi_transmit_single_command(&init_task); // enable gyro

    init_task.command = (uint8_t *)&gyro_setup_commands[2];
    IMU::spi_transmit_single_command(&init_task); // enable axes



    // setup accel
    init_task.command = (uint8_t *)accel_setup_commands;
    init_task.device_type = ACCELEROMETER;

    IMU::spi_transmit_single_command(&init_task); // enable axes

    init_task.command = (uint8_t *)&accel_setup_commands[2];
    IMU::spi_transmit_single_command(&init_task); // exit powerdown

    init_task.command = (uint8_t *)&accel_setup_commands[4];
    IMU::spi_transmit_single_command(&init_task); 

    // setup mag
    init_task.command = (uint8_t *)mag_setup_commands;
    init_task.device_type = MAGNETOMETER;
    IMU::spi_transmit_single_command(&init_task); // mag setup part 1
    
    init_task.command = (uint8_t *)&mag_setup_commands[2];
    IMU::spi_transmit_single_command(&init_task); // mag setup part 2

    // general setup
    IMU::multi_read_enabled = false;
    IMU::toggle_multi_read();
}

void IMU::read_gyroscope_task(void *pvParameters) {
    imu_data_t *imu_readings = (imu_data_t *)pvParameters;
    while (1) {
        IMU::read_gyroscope(imu_readings);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void IMU::read_accelerometer_task(void *pvParameters) {
    imu_data_t *imu_readings = (imu_data_t *)pvParameters;
    while (1) {
        IMU::read_accelerometer(imu_readings);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void IMU::read_magnetometer_task(void *pvParameters) {
    imu_data_t *imu_readings = (imu_data_t *)pvParameters;
    while (1) {
        IMU::read_magnetometer(imu_readings);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void IMU::read_magnetometer(imu_data_t* imu_readings) {
    uint8_t mag_data[7];

    spi_transmit_single_command_task_s get_mag_data = {
        .command = (uint8_t[]){OUT_X_L_M},
        .command_length = 1,
        .dummy_length = 6,
        .device_type = MAGNETOMETER,
        .response = mag_data,
        .read_active_high = 1,
        .multi_read_active_high = 1
    };

    IMU::spi_transmit_single_command(&get_mag_data);

    IMU::package_data(mag_data + 1, &imu_readings->raw_imu_readings.mag);
    IMU::convert_raw_data(&imu_readings->raw_imu_readings, &imu_readings->converted_imu_eadings, MAGNETOMETER);
}

void IMU::read_gyroscope(imu_data_t* imu_readings) {
    if (!IMU::multi_read_enabled) {
        IMU::toggle_multi_read();
        IMU::multi_read_enabled = true;
    }
    uint8_t gyro_data[7];

    spi_transmit_single_command_task_s get_gyro_data = {
        .command = (uint8_t[]){OUT_X_L_G},
        .command_length = 1,
        .dummy_length = 6,
        .device_type = GYROSCOPE,
        .response = gyro_data,
        .read_active_high = 1,
        .multi_read_active_high = 1 // doesn't matter for gyro
    };
    IMU::spi_transmit_single_command(&get_gyro_data);

    IMU::package_data(gyro_data + 1, &imu_readings->raw_imu_readings.gyro);
    IMU::convert_raw_data(&imu_readings->raw_imu_readings, &imu_readings->converted_imu_eadings, GYROSCOPE);
}

void IMU::read_accelerometer(imu_data_t* imu_readings) {
    if (!IMU::multi_read_enabled) {
        IMU::toggle_multi_read();
        IMU::multi_read_enabled = true;
    }

    uint8_t accel_data[7];
    spi_transmit_single_command_task_s get_accel_data = {
        .command = (uint8_t[]){OUT_X_L_XL},
        .command_length = 1,
        .dummy_length = 6,
        .device_type = ACCELEROMETER,
        .response = accel_data,
        .read_active_high = 1,
        .multi_read_active_high = 1 // doesn't matter for accel
    };
    IMU::spi_transmit_single_command(&get_accel_data);
    IMU::package_data(accel_data, &imu_readings->raw_imu_readings.accel);
    IMU::convert_raw_data(&imu_readings->raw_imu_readings, &imu_readings->converted_imu_eadings, ACCELEROMETER);
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
        .read_active_high = 0,
        .multi_read_active_high = 1 // doesn't matter for gyro
    };
    IMU::spi_transmit_single_command(&multi_read);
    IMU::multi_read_enabled = !IMU::multi_read_enabled;
    if (IMU::multi_read_enabled) {
        printf("Multi-read enabled\n");
    } else {
        printf("Multi-read disabled\n");
    }
}

void IMU::print_imu_task(void *pvParameters) {
    imu_data_t *imu_readings = (imu_data_t *)pvParameters;
    while (1) {
        printf("Accel: X=%f Y=%f Z=%f | Gyro: X=%f Y=%f Z=%f | Mag: X=%f Y=%f Z=%f\n",
            imu_readings->converted_imu_eadings.accel.x, imu_readings->converted_imu_eadings.accel.y, imu_readings->converted_imu_eadings.accel.z,
            imu_readings->converted_imu_eadings.gyro.x, imu_readings->converted_imu_eadings.gyro.y, imu_readings->converted_imu_eadings.gyro.z,
            imu_readings->converted_imu_eadings.mag.x, imu_readings->converted_imu_eadings.mag.y, imu_readings->converted_imu_eadings.mag.z);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// In app_main(), after initializing and starting other tasks: