#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>


#include "IMU.h"

extern "C" void app_main() {
	spi_init();
	IMU::init();

	// xTaskCreate(IMU::spi_transmit_single_command_task, "IMU Task", 4096, &get_gyro_data, 5, NULL);
	xTaskCreate(IMU::read_gyroscope_task, "Read Gyro Task", 4096, NULL, 5, NULL);
	xTaskCreate(IMU::read_accelerometer_task, "Read Accel Task", 4096, NULL, 5, NULL);
	xTaskCreate(IMU::read_magnetometer_task, "Read Mag Task", 4096, NULL, 5, NULL);

	xTaskCreate(IMU::package_data_task, "Package Data Task", 4096, NULL, 5, NULL);
	xTaskCreate(IMU::print_imu_task, "Print IMU Task", 4096, NULL, 5, NULL);
}