#include "SPI.h"

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} sensor_data;

typedef struct {
    // float temperature;
    sensor_data accel;
    sensor_data gyro;
    sensor_data mag;
} imu_data;


void read_gyro_task(void *pvParameters);
void read_magnetometer_task(void *pvParameters);
void package_data(uint8_t **raw_data, sensor_data* sensor);