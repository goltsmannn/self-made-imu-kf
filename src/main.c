#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>

#include "IMU.h"
uint8_t command = WHO_AM_I_M;
spi_task_data_t mag_task_data = {
	.command = &command,
	.command_length = 1,
	.dummy_length = 1,
	.device_type = ACCELEROMETER,
	.response = NULL,
	.read_active_high = 1
};

void app_main() {
	printf("Initializing IMU SPI...\r\n");
	imu_spi_init();
	xTaskCreate(imu_spi_transfer_task, "IMU Task", 4096, &mag_task_data, 5, NULL);
}