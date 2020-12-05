#include "MPU6050.h"

uint8_t Mpu6050_Read(unsigned char address)
{
		TX[0] = address;
		HAL_I2C_Master_Transmit(&hi2c1, mpu6050, TX, 1, 50);
		HAL_I2C_Master_Receive(&hi2c1, mpu6050|0x01, &RX, 1, 50);
		return RX;
}
void Mpu6050_Write(unsigned char address,unsigned char Data)
{
		TX[0] = address;
		TX[1] = Data;
		HAL_I2C_Master_Transmit(&hi2c1, mpu6050, TX, 2, 50);
	}

void Mpu6050_Init()
{
	Mpu6050_Write(PWR_MGMT_1, 0x00);  	// internal 8MHz, disabled SLEEP mode, disable CYCLE mode  
	Mpu6050_Write(SMPLRT_DIV, 0x07);		//sample rate: 8khz
	Mpu6050_Write(CONFIG, 0x06);		 		//DLPF disable
	Mpu6050_Write(GYRO_CONFIG, 0x18);  	//full scale range mode 3 +-2000do/s
	Mpu6050_Write(ACCEL_CONFIG, 0x01); 	//full scale range mode 1 +-4g
}

int16_t GetData(unsigned char address)
{
	char H,L;
	H=Mpu6050_Read(address);
	L=Mpu6050_Read(address+1);
	return (H<<8)+L;   
}

void GetAcc()
{
	Ax = GetData(ACCEL_XOUT_H);
	Ay = GetData(ACCEL_YOUT_H);
	Az = GetData(ACCEL_ZOUT_H);
}

void GetGyro()
{
	Gx = GetData(GYRO_XOUT_H);
	Gy = GetData(GYRO_YOUT_H);
//	Gz = GetData(GYRO_ZOUT_H);
}

void Gyro_Angle()
{
    GetGyro();
    
    GYRO_X_RATE = (double)(Gx-GYRO_X_OFFSET)/gyro_sensitivity;
    GYRO_Y_RATE = (double)(Gy-GYRO_Y_OFFSET)/gyro_sensitivity;
//    GYRO_Z_RATE = (double)(Gz-GYRO_Z_OFFSET)/gyro_zsensitivity;
            	
    GYRO_X_ANGLE = ANGLE_X_REAL + GYRO_X_RATE*dt;
    GYRO_Y_ANGLE = ANGLE_Y_REAL + GYRO_Y_RATE*dt;
//    GYRO_Z_ANGLE += GYRO_Z_RATE*dt;
}

void Accel_Angle()
{	
    GetAcc();
      	
    ACCEL_X_ANGLE = 57.295*atan((double)Ay/ sqrt(pow((double)Az,2)+pow((double)Ax,2)));
    ACCEL_Y_ANGLE = 57.295*atan((double)-Ax/ sqrt(pow((double)Az,2)+pow((double)Ay,2)));  
	
		ANGLE_X_REAL = 0.04*ACCEL_X_ANGLE +0.96*GYRO_X_ANGLE;
		ANGLE_Y_REAL = 0.04*ACCEL_Y_ANGLE +0.96*GYRO_Y_ANGLE;
}

void Calibrate_Gyros()
{
	int x;
	GYRO_X_OFFSET_SUM = 0;
	GYRO_Y_OFFSET_SUM = 0;
//	GYRO_Z_OFFSET_SUM = 0;

	for(x = 0; x<5000; x++)
	{
		GetGyro();
		GYRO_X_OFFSET_SUM += Gx; 
		GYRO_Y_OFFSET_SUM += Gy;
//		GYRO_Z_OFFSET_SUM += Gz;  

		if(x%700 == 0)
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	}	
	
	GYRO_X_OFFSET = (double)GYRO_X_OFFSET_SUM/5000;
	GYRO_Y_OFFSET = (double)GYRO_Y_OFFSET_SUM/5000;
//	GYRO_Z_OFFSET = (double)GYRO_Z_OFFSET_SUM/5000;
}
