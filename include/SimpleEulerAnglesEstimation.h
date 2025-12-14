#ifndef SIMPLE_EULER_ANGLES_ESTIMATION_H
#define SIMPLE_EULER_ANGLES_ESTIMATION_H

#include "IMU.h"
#include <cmath>

struct EulerAngles_t {
    float roll;  
    float pitch; 
    float yaw;  
};

class EulerAngles {

    public:
        static EulerAngles_t euler_angles_data;
        static SemaphoreHandle_t euler_angles_ready_semaphore;
        static void computeEulerAnglesTask(void *pvParameters);
        static void printEulerAnglesTask(void *pvParameters);
    private:
        static void computeRollPitch(float ax, float ay, float az, float &roll, float &pitch);
        static void computeYaw(float mx, float my, float mz, float roll, float pitch, float &yaw);
};

#endif // SIMPLE_EULER_ANGLES_ESTIMATION_H