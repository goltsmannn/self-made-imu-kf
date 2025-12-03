I will be implementing a self-made IMU with a 9-DOF set of sensors: accelerometer, magnetometer, and gyro (packages as LSM9DS1).

The goal:
1) FREERTOS allowing for simultaneous sensor reads and shared state updates
2) Extended Kalman Filter to fuse sensor measurements
3) Automatic calibration at reset
