// Simple LED blink for ESP32 (AZ-Delivery Dev Kit v4)
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
# include "FreeRTOSConfig.h"
#include "IMU.h"



void app_main() {
	imu_spi_init();
	xTaskCreate(read_magnetometer_task, "IMU Task", 4096, NULL, 5, NULL);
}