#include "SPI.h"
#include "etl/singleton.h"
#include <stdint.h>

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
} DeviceType; 

struct spi_transmit_single_command_task_s {
    uint8_t *command;
    uint8_t command_length;
    uint8_t dummy_length;
    DeviceType device_type;
    uint8_t *response;
    uint8_t read_active_high;
    uint8_t multi_read_active_high;
};




class IMU : public etl::singleton<IMU> {
public:

    static void spi_transmit_single_command(spi_transmit_single_command_task_s *task_data);
    static void spi_transmit_single_command_task(void *pvParameters);

    static void read_gyroscope_task(void *pvParameters);
    static void read_accelerometer_task(void *pvParameters);
    static void read_magnetometer_task(void *pvParameters);

    static void read_gyroscope(imu_data* imu_readings);
    static void read_accelerometer(imu_data* imu_readings);
    static void read_magnetometer(imu_data* imu_readings);

    static void init();
    
    static void set_sensor(DeviceType type, sensor_data sensor);

    static void print_imu_task(void *pvParameters);

private:
    static bool multi_read_enabled;
    static void package_data(uint8_t *raw_data, sensor_data* sensor);
    static void toggle_multi_read();
    // IMU() {}
};



// void imu_send_multiple_commands(imu_multi_read_task_data_t *task_data);

// void read_mult_task(void *pvParameters);

