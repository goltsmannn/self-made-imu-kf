#ifndef SPI_H
#define SPI_H

#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "lsm9ds1_registers.h"


#define HSPI_MISO_M_PIN GPIO_NUM_12
#define HSPI_MISO_AG_PIN GPIO_NUM_36
#define HSPI_MOSI_PIN GPIO_NUM_13
#define HSPI_SCLK_PIN GPIO_NUM_14
#define HSPI_CS_M_PIN GPIO_NUM_33
#define HSPI_CS_AG_PIN GPIO_NUM_32


extern const spi_bus_config_t magnetometer_spi_config;
extern const spi_bus_config_t accelerometer_gyroscope_spi_config;
extern const spi_device_interface_config_t magnetometer_spi_device_config;
extern const spi_device_interface_config_t accelerometer_gyroscope_spi_device_config;

extern spi_device_handle_t magnetometer_spi_device_handle;
extern spi_device_handle_t accelerometer_gyroscope_spi_device_handle;

void imu_spi_init();
uint8_t spi_read_magnetometer(uint8_t address);
void spi_read_accelerometer_gyroscope(uint8_t address);




#endif // SPI_H