#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>


#include "IMU.h"
uint8_t command = OUT_X_L_G;
uint8_t rx_dummy[4];
spi_transmit_single_command_task_s get_gyro_data = {
	.command = &command,
	.command_length = 1,
	.dummy_length = 1,
	.device_type = GYROSCOPE,
	.response = rx_dummy,
	.read_active_high = 1
};

extern "C" void app_main() {
	imu_data imu_readings = {};
	spi_init();
	IMU::init();

	// xTaskCreate(IMU::spi_transmit_single_command_task, "IMU Task", 4096, &get_gyro_data, 5, NULL);
	xTaskCreate(IMU::read_gyroscope_task, "Read Gyro Task", 4096, &imu_readings, 5, NULL);
	xTaskCreate(IMU::read_accelerometer_task, "Read Accel Task", 4096, &imu_readings, 5, NULL);
	xTaskCreate(IMU::print_imu_task, "Print IMU Task", 2048, &imu_readings, 5, NULL);
}