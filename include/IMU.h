#include "SPI.h"
#include "etl/singleton.h"
#include <stdint.h>



// Data from Table 3
#define ACCEL_SENSITIVITY 0.061  // mg/LSB for ±2g [21]
#define GYRO_SENSITIVITY 8.75    // mdps/LSB for ±245 dps [21]
#define MAG_SENSITIVITY 0.14     // mgauss/LSB for ±4 gauss [21]

#define GYRO_CALIBRATION_SAMPLES 100

#define RAW_TO_ACCEL_G(raw) ((float)(raw) * ACCEL_SENSITIVITY / 1000.0f)
#define RAW_TO_GYRO_DPS(raw) ((float)(raw) * GYRO_SENSITIVITY / 1000.0f)
#define RAW_TO_MAG_GAUSS(raw) ((float)(raw) * MAG_SENSITIVITY / 1000.0f)

#define ACCEL_BIT (1 << 0)
#define GYRO_BIT  (1 << 1)
#define MAG_BIT   (1 << 2)

#define BATCH_SIZE 10

#define ACCEL_GYRO_INTERRUPT_PIN (GPIO_NUM_16)
#define MAG_INTERRUPT_PIN (GPIO_NUM_17)

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} sensor_data;

typedef struct {
    float x;
    float y;
    float z;
} converted_sensor_data;

typedef struct {
    // float temperature;
    sensor_data accel;
    sensor_data gyro;
    sensor_data mag;
} raw_imu_data;

typedef struct {
    // float temperature;
    converted_sensor_data accel;
    converted_sensor_data gyro;
    converted_sensor_data mag;
} converted_imu_data;

typedef struct {
    raw_imu_data raw_imu_readings;
    converted_imu_data converted_imu_readings;
} imu_data_t;

typedef enum  {
    ACCELEROMETER = 0,
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

struct SensorQueueEntity {
    DeviceType device_type;
    uint8_t data[6];
};



class IMU : public etl::singleton<IMU> {
public:
    static imu_data_t shared_imu_data;
    static converted_sensor_data calibrated_gyro_offset;

    static QueueHandle_t sensor_data_queue;
    static QueueHandle_t batch_data_queue;
    
    static EventGroupHandle_t print_ready_event_group;
    static SemaphoreHandle_t shared_imu_data_mutex;
    static SemaphoreHandle_t accelGyroDRDYSemaphore;
    static SemaphoreHandle_t magDRDYSemaphore;
    static SemaphoreHandle_t batchReadySemaphore;

    static bool multi_read_enabled;


    static void spi_transmit_single_command(spi_transmit_single_command_task_s *task_data);
    static void spi_transmit_single_command_task(void *pvParameters);


    static void read_gyroscope_task(void *pvParameters);
    static void read_accelerometer_task(void *pvParameters);
    static void read_magnetometer_task(void *pvParameters);

    static void package_data_task(void *pvParameters);
    static void print_imu_task(void *pvParameters);


    static void read_gyroscope();
    static void read_accelerometer();
    static void read_magnetometer();

    static void init();
    
    static void set_sensor(DeviceType type, sensor_data sensor);

    static void print_imu();

private:
    static void package_data();
    static void calibrate_gyroscope();
    static void toggle_multi_read();
    static void configure_interrupts();
    // IMU() {}
};



// void imu_send_multiple_commands(imu_multi_read_task_data_t *task_data);

// void read_mult_task(void *pvParameters);

