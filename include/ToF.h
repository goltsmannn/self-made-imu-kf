#ifndef TOF_H
#define TOF_H
//! DEPRECATED - for reference only

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_MASTER_ADDR 0x29
#define I2C_MASTER_NUM I2C_NUM_0

void tof_init();
void tof_read();

#endif // TOF_H