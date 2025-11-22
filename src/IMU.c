#include "IMU.h"

void package_data(uint8_t **raw_data, sensor_data* sensor) {
    sensor->x = (int16_t)((raw_data[0][1] << 8) | raw_data[0][0]);
    sensor->y = (int16_t)((raw_data[1][1] << 8) | raw_data[1][0]);
    sensor->z = (int16_t)((raw_data[2][1] << 8) | raw_data[2][0]);
}

void read_gyro_task(void *pvParameters) {
    uint8_t addresses[3][2] = {
        {OUT_X_H_G, OUT_X_L_G}, // X-axis
        {OUT_Y_H_G, OUT_Y_L_G}, // Y-axis
        {OUT_Z_H_G, OUT_Z_L_G}  // Z-axis
    };

    uint8_t raw_data[3] = {0};

    for (int axis = 0; axis < 3; axis++) {
        for (int byte = 0; byte < 2; byte++) {
            raw_data[axis] <<= 8;
            uint8_t address = addresses[axis][byte];
            uint8_t responspi_read_accelerometer_gyroscope(address);
            
        }
    }
    // spi_read_accelerometer_gyroscope(address);

}

void read_magnetometer_task(void *pvParameters) {
    uint8_t addresses[3][2] = {
        {OUT_X_H_M, OUT_X_L_M}, // X-axis
        {OUT_Y_H_M, OUT_Y_L_M}, // Y-axis
        {OUT_Z_H_M, OUT_Z_L_M}  // Z-axis
    };

    uint8_t raw_data[3][2] = {0};

    while (1) {
        for (int axis = 0; axis < 3; axis++) {
            for (int byte = 0; byte < 2; byte++) {
                uint8_t address = addresses[axis][byte];
                uint8_t response = spi_read_magnetometer(address);
                raw_data[axis][byte] = response;
            }
        }

        sensor_data mag_data;
        package_data(raw_data, &mag_data);
        printf("Magnetometer Data - X: %d, Y: %d, Z: %d\r\n", mag_data.x, mag_data.y, mag_data.z);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Read every 1000 ms
    }
}