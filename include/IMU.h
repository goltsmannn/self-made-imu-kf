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

typedef enum  {
    ACCELEROMETER,
    GYROSCOPE,
    MAGNETOMETER
} DeviceType; //

typedef struct {
    uint8_t *command;
    uint8_t command_length;
    uint8_t dummy_length;
    DeviceType device_type;
    uint8_t *response;
    uint8_t read_active_high;
} spi_task_data_t;

void imu_spi_transfer_task(void *pvParameters);

void imu_spi_transfer(spi_task_data_t *task_data);

void package_data(uint8_t (*raw_data)[2], sensor_data* sensor);