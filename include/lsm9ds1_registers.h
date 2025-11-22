// lsm9ds1_regs.h
#pragma once

#define WHO_AM_I_XG      0x0F   // Accel/Gyro
#define WHO_AM_I_M       0x0F   // Magnetometer (same address on mag)

// Accel and gyro (control registers)
#define CTRL_REG1_G      0x10
#define CTRL_REG2_G      0x11
#define CTRL_REG3_G      0x12
#define CTRL_REG5_XL     0x1F
#define CTRL_REG6_XL     0x20
#define CTRL_REG7_XL     0x21
#define CTRL_REG8        0x22
#define CTRL_REG9        0x23

// Magnetometer (control registers)
#define CTRL_REG1_M      0x20
#define CTRL_REG2_M      0x21
#define CTRL_REG3_M      0x22
#define CTRL_REG4_M      0x23
#define CTRL_REG5_M      0x24

// Output registers – Accel
#define OUT_X_L_XL       0x28
#define OUT_X_H_XL       0x29
#define OUT_Y_L_XL       0x2A
#define OUT_Y_H_XL       0x2B
#define OUT_Z_L_XL       0x2C
#define OUT_Z_H_XL       0x2D

// Output registers – Gyro
#define OUT_X_L_G        0x18
#define OUT_X_H_G        0x19
#define OUT_Y_L_G        0x1A
#define OUT_Y_H_G        0x1B
#define OUT_Z_L_G        0x1C
#define OUT_Z_H_G        0x1D

// Output registers – Magnetometer
#define OUT_X_L_M        0x28
#define OUT_X_H_M        0x29
#define OUT_Y_L_M        0x2A
#define OUT_Y_H_M        0x2B
#define OUT_Z_L_M        0x2C
#define OUT_Z_H_M        0x2D

// Optional: status registers
#define STATUS_REG_XG    0x17
#define STATUS_REG_M     0x27
