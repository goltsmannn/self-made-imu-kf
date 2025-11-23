#ifndef SPI_H
#define SPI_H

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <string.h>

#include "lsm9ds1_registers.h"


#define HSPI_MISO_PIN GPIO_NUM_19
#define HSPI_MOSI_PIN GPIO_NUM_23
#define HSPI_SCLK_PIN GPIO_NUM_18
#define HSPI_CS_M_PIN GPIO_NUM_33
#define HSPI_CS_AG_PIN GPIO_NUM_32


extern  spi_bus_config_t spi_config;
extern  spi_device_interface_config_t magnetometer_spi_device_config;
extern  spi_device_interface_config_t accelerometer_gyroscope_spi_device_config;

extern spi_device_handle_t magnetometer_spi_device_handle;
extern spi_device_handle_t accelerometer_gyroscope_spi_device_handle;

void spi_init();
void spi_transmit(uint8_t *tx, uint8_t *rx, spi_device_handle_t device, size_t length);




#endif // SPI_H