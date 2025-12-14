#include <cmath>

#include "SimpleEulerAnglesEstimation.h"

EulerAngles_t EulerAngles::euler_angles_data = {};
SemaphoreHandle_t EulerAngles::euler_angles_ready_semaphore = xSemaphoreCreateBinary();

void EulerAngles::computeRollPitch(float ax, float ay, float az, float &roll, float &pitch) {
    roll  = atan2(-ax, sqrt(ay*ay + az*az));  
    pitch = atan2(ay, az);                    
}

void EulerAngles::computeYaw(float mx, float my, float mz, float roll, float pitch, float &yaw) {
    float mx2 = mx * cos(pitch) - mz * sin(pitch);
    float my2 = mx * sin(roll) * sin(pitch) + my * cos(roll) + mz * sin(roll) * cos(pitch);
    yaw = atan2(-my2, mx2);
}

void EulerAngles::computeEulerAnglesTask(void *pvParameters) {
    for (;;) {
        xEventGroupWaitBits(IMU::print_ready_event_group,
            ACCEL_BIT | MAG_BIT,
            pdTRUE, pdTRUE, portMAX_DELAY);

        xSemaphoreTake(IMU::shared_imu_data_mutex, portMAX_DELAY);
        converted_imu_data copy_imu_data = IMU::shared_imu_data.converted_imu_readings; // deep copy
        xSemaphoreGive(IMU::shared_imu_data_mutex);

        computeRollPitch(copy_imu_data.accel.x, copy_imu_data.accel.y, copy_imu_data.accel.z,
            EulerAngles::euler_angles_data.roll,
            EulerAngles::euler_angles_data.pitch);

        computeYaw(copy_imu_data.mag.x, copy_imu_data.mag.y, copy_imu_data.mag.z,
            EulerAngles::euler_angles_data.roll,
            EulerAngles::euler_angles_data.pitch,
            EulerAngles::euler_angles_data.yaw);
        xSemaphoreGive(EulerAngles::euler_angles_ready_semaphore);
    }
}

void EulerAngles::printEulerAnglesTask(void *pvParameters) {
    for (;;) {
        xSemaphoreTake(EulerAngles::euler_angles_ready_semaphore, portMAX_DELAY);
        printf("Roll: %f, Pitch: %f, Yaw: %f\n",
            EulerAngles::euler_angles_data.roll * (180.0f / M_PI),
            EulerAngles::euler_angles_data.pitch * (180.0f / M_PI),
            EulerAngles::euler_angles_data.yaw * (180.0f / M_PI));
    }
}
