#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f4xx_hal.h"
#include "math.h"

extern int16_t Ax, Ay, Az, Gx, Gy, Gz;
extern double GYRO_X_RATE, GYRO_Y_RATE, GYRO_Z_RATE, GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET, GYRO_X_ANGLE, GYRO_Y_ANGLE, GYRO_Z_ANGLE, ACCEL_X_ANGLE, ACCEL_Y_ANGLE, ANGLE_X_REAL, ANGLE_Y_REAL, dt;
extern uint8_t TX[2], RX;
extern I2C_HandleTypeDef hi2c1;
extern long GYRO_X_OFFSET_SUM, GYRO_Y_OFFSET_SUM, GYRO_Z_OFFSET_SUM;


#define   SMPLRT_DIV      0x19   //0x07(125Hz)
#define   CONFIG         0x1A   //0x06(5Hz)
#define   GYRO_CONFIG      0x1B   //0x18(不自检，2000deg/s)
#define   ACCEL_CONFIG   0x1C   //0x01(不自检，2G，5Hz)
#define   ACCEL_XOUT_H   0x3B
#define   ACCEL_XOUT_L   0x3C
#define   ACCEL_YOUT_H   0x3D
#define   ACCEL_YOUT_L   0x3E
#define   ACCEL_ZOUT_H   0x3F
#define   ACCEL_ZOUT_L   0x40
#define   TEMP_OUT_H      0x41
#define   TEMP_OUT_L      0x42
#define   GYRO_XOUT_H      0x43
#define   GYRO_XOUT_L      0x44   
#define   GYRO_YOUT_H      0x45
#define   GYRO_YOUT_L      0x46
#define   GYRO_ZOUT_H      0x47
#define   GYRO_ZOUT_L      0x48
#define   PWR_MGMT_1      0x6B   //
#define   WHO_AM_I         0x75   //
#define mpu6050 0xD0  //dia chi cua cam bien mpu6050

#define gyro_sensitivity 16.4 


uint8_t Mpu6050_Read(unsigned char address);
void Mpu6050_Write(unsigned char address,unsigned char Data);
void Mpu6050_Init();
int16_t GetData(unsigned char address);
void GetAcc();
void GetGyro();
void Gyro_Angle();
void Accel_Angle();
void Calibrate_Gyros();

#endif
