#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>


#include "IMU.h"
#include "SimpleEulerAnglesEstimation.h"
#include "SimpleEulerAnglesEstimation.h"

extern "C" void app_main() {
	spi_init();
	// important to have it here before spi for queue to work
	xTaskCreate(IMU::package_data_task, "Package Data Task", 4096, NULL, 5, NULL);
	// xTaskCreate(IMU::print_imu_task, "Print IMU Task", 4096, NULL, 2, NULL);
	xTaskCreate(EulerAngles::computeEulerAnglesTask, "Compute Euler Angles Task", 4096, NULL, 3, NULL);
	xTaskCreate(EulerAngles::printEulerAnglesTask, "Print Euler Angles Task", 4096, NULL, 2, NULL);
	vTaskDelay(pdMS_TO_TICKS(200)); // wait for SPI to stabilize
	IMU::init();

	// xTaskCreate(IMU::spi_transmit_single_command_task, "IMU Task", 4096, &get_gyro_data, 5, NULL);
	xTaskCreate(IMU::read_gyroscope_task, "Read Gyro Task", 4096, NULL, 4, NULL);

	// TODO: THIS IS A TEMPORARY FIX AS I ONLY HAVE ONE WIRE FOR TWO INTERRUPTS (OUT OF JUMPERS)
	// xTaskCreate(IMU::read_accelerometer_task, "Read Accel Task", 4096, NULL, 4, NULL);
	xTaskCreate(IMU::read_magnetometer_task, "Read Mag Task", 4096, NULL, 4, NULL);

}