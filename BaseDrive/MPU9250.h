/**
  ******************************************************************************
  * @file    MPU9250.h
  * @author  
  * @version V1.0
  * @date    27-January-2015
  * @brief   Header file for MPU9250.c module.
  
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WAVESHARE SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2014 Waveshare</center></h2>
  ******************************************************************************
  */


#ifndef __MPU9250_H
#define __MPU9250_H

#include "Public_StdTypes.h"
#include "I2C.h"
//#include "System.h"
#include "IMU.H"

// define MPU9250 register address
//****************************************
#define	SMPLRT_DIV		0x19	//Sample Rate Divider. Typical values:0x07(125Hz) 1KHz internal sample rate
#define	CONFIG				0x1A	//Low Pass Filter.Typical values:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//Gyro Full Scale Select. Typical values:0x10(1000dps)
#define	ACCEL_CONFIG	0x1C	//Accel Full Scale Select. Typical values:0x01(2g)

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08


#define	PWR_MGMT_1		0x6B	//Power Management. Typical values:0x00(run mode)
#define	WHO_AM_I		  0x75	//identity of the device


#define	GYRO_ADDRESS   0xD0	  //Gyro and Accel device address
#define MAG_ADDRESS    0x18   //compass device address
#define ACCEL_ADDRESS  0xD0 

#define ADDRESS_AD0_LOW     0xD0 //address pin low (GND), default for InvenSense evaluation board
#define ADDRESS_AD0_HIGH    0xD1 //address pin high (VCC)
#define DEFAULT_ADDRESS     GYRO_ADDRESS
#define WHO_AM_I_VAL				0x73 //identity of MPU9250 is 0x71. identity of MPU9255 is 0x73.


typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}MPU9250_TypeDef;

typedef struct
{
	int16_t X_Off_Err;
	int16_t Y_Off_Err;
	int16_t Z_Off_Err;
}MPU9250_TypeDef_Off;

typedef struct
{
	uint8_t Index;
	int16_t AvgBuffer[8];
}MPU9250_AvgTypeDef;

extern int16_t magn[3];
extern int16_t accel[3], gyro[3];

void MPU9250_Init(void);
void MPU9250_READ_ACCEL(void);
void MPU9250_READ_GYRO(void);
void MPU9250_READ_MAG(void);
bool MPU9250_Check(void);
void MPU9250_CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal);
void MPU9250_InitGyrOffset(void);
#endif
