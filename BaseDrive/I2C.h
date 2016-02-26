/**
  ******************************************************************************
  * @file    I2C.h
  * @author  Waveshare Team
  * @version V1.0
  * @date    29-August-2014
  * @brief   This file contains all the functions prototypes for the I2C firmware 
  *          library.

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
  ******************************************************************************
  */


#ifndef _I2C_H_
#define _I2C_H_

#include "Public_StdTypes.h"


#define I2C_SCL_PIN  GPIO_Pin_10
#define I2C_SDA_PIN  GPIO_Pin_11

#define I2C_SCL_Set()  GPIO_WriteBit(GPIOB, I2C_SCL_PIN, Bit_SET)
#define I2C_SCL_Clr()  GPIO_WriteBit(GPIOB, I2C_SCL_PIN, Bit_RESET)

#define I2C_SDA_Set()  GPIO_WriteBit(GPIOB, I2C_SDA_PIN, Bit_SET)
#define I2C_SDA_Clr()  GPIO_WriteBit(GPIOB, I2C_SDA_PIN, Bit_RESET)

#define I2C_SDA_Get()  GPIO_ReadInputDataBit(GPIOB, I2C_SDA_PIN)

#ifndef I2C_Direction_Transmitter
	#define  I2C_Direction_Transmitter      ((uint8_t)0x00)
#endif

#ifndef I2C_Direction_Receiver
	#define  I2C_Direction_Receiver         ((uint8_t)0x01)
#endif

enum
{
	I2C_SDA_IN,
	I2C_SDA_OUT
};

enum
{
	I2C_ACK,
	I2C_NACK
};

extern void I2C_SDAMode(uint8_t Mode);
extern void I2C_Start(void);
extern void I2C_Stop(void);
extern bool I2C_WaiteForAck(void);
extern void I2C_Ack(void);
extern void I2C_NAck(void);
extern bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data);
extern bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data);
extern void I2C_WriteByte(uint8_t Data);
extern uint8_t I2C_ReadByte(uint8_t Ack);
extern void I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data);
extern uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr);
extern bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);
extern bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);

extern void I2Cx_Init(void);
extern void Delay_us(uint16_t Time);
extern void Delay_ms(uint16_t Time);

#endif

/******************* (C) COPYRIGHT 2014 Waveshare *****END OF FILE*******************/

