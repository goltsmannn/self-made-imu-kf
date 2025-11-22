#include "IMU.h"

void package_data(uint8_t (*raw_data)[2], sensor_data* sensor) {
    sensor->x = (int16_t)((raw_data[0][1] << 8) | raw_data[0][0]);
    sensor->y = (int16_t)((raw_data[1][1] << 8) | raw_data[1][0]);
    sensor->z = (int16_t)((raw_data[2][1] << 8) | raw_data[2][0]);
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

    spi_transmit(tx, task_data->response, device, task_data->command_length + task_data->dummy_length);
}

