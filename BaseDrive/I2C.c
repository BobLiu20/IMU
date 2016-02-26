/**
  ******************************************************************************
  * @file    I2C.c
  * @author  Waveshare Team
  * @version V1.0
  * @date    29-August-2014
  * @brief   This file provides all the I2C firmware functions.

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


#include "I2C.h"

/**
  * @brief  Initializes I2C GPIO
  * @param  None
  * @retval None
  */

void I2C_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //     GPIO_Mode_Out_PP
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_SCL_Set();
	I2C_SDA_Set();
	//Delay_ms(10);
}

/**
  * @brief  Initializes I2C 
  * @param  None
  * @retval None
  */

void I2Cx_Init(void)
{
	I2C_GPIOInit();
}

/**
  * @brief  Configures SDA in input/output mode 
  * @param  Mode
 *                   @arg 0: SDA in input mode
  *                  @arg 1: SDA in output mode
  * @retval None
  */

void I2C_SDAMode(uint8_t Mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;

	if(Mode)
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	}
	else
	{
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	}
		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	Delay_us(10);
}

/**
  * @brief   Generates I2C communication START condition.
  *         
  * @param  None
  *         
  * @retval  None
**/

void I2C_Start(void)
{
	I2C_SDAMode(I2C_SDA_OUT);
	I2C_SDA_Set();
	Delay_us(1);
	I2C_SCL_Set();
	Delay_us(1);
	I2C_SDA_Clr();
	Delay_us(1);
	I2C_SCL_Clr();
	Delay_us(1);
}

/**
  * @brief  Generates I2C communication STOP condition.
  *         
  * @param  None
  *         
  * @retval  None
**/

void I2C_Stop(void)
{
	I2C_SCL_Clr();
	I2C_SDA_Clr();
	Delay_us(1);
	I2C_SCL_Set();
	Delay_us(1);
	I2C_SDA_Set();
	Delay_us(1);
}

/**
  * @brief  I2C is waiting for an ACK
  *         
  * @param  None
  *         
  * @retval  I2C ACK status
  *              @arg 0:I2C has not waited an ACK
  *              @arg 1:I2C has  waited an ACK
**/

bool I2C_WaiteForAck(void)
{
	uint8_t Retry = 0;
	
	I2C_SCL_Clr();
	Delay_us(1);
	I2C_SCL_Set();
	I2C_SDAMode(I2C_SDA_IN);
	while(I2C_SDA_Get())
	{
		if(++ Retry > 200)
		{
			I2C_Stop();
			return false;
		}
	}
	I2C_SDAMode(I2C_SDA_OUT);
	I2C_SCL_Clr();
	Delay_us(1);
	
	return true;
}

/**
  * @brief  I2C responds an ACK
  *         
  * @param  None
  *         
  * @retval  None
**/

void I2C_Ack(void)
{
	I2C_SCL_Clr();
	I2C_SDAMode(I2C_SDA_OUT);
	I2C_SDA_Clr();
	Delay_us(1);
	I2C_SCL_Set();
	Delay_us(1);
	I2C_SCL_Clr();
}

/**
  * @brief  I2C responds an NACK
  *         
  * @param  None
  *         
  * @retval  None
**/
void I2C_NAck(void)
{
	I2C_SCL_Clr();
	I2C_SDAMode(I2C_SDA_OUT);
	I2C_SDA_Set();
	Delay_us(1);
	I2C_SCL_Set();
	Delay_us(1);
	I2C_SCL_Clr();
}

/**
  * @brief  Write one bit to I2C bus
  *         
  * @param DevAddr: The address byte of the slave device
  * @param RegAddr: The address byte of  register of the slave device
  * @param BitNum:  which bit of the byte that  would be writen to
  * @param Data: bit value(0 or 1)
  *         
  * @retval  true: writen one bit succeed
**/

bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
    uint8_t Dat;
    
    Dat =I2C_ReadOneByte(DevAddr, RegAddr);
    Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}


/**
  * @brief  Write couplesof bits to I2C bus
  *         
  * @param DevAddr: The address byte of the slave device
  * @param RegAddr: The address byte of  register of the slave device
  * @param BitStart: the first bit of the several bits would be to writen
  * @param Data: the data would be writen      
  * @retval  writen couples of bits succeed
**/

bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{

    uint8_t Dat, Mask;
    
	Dat = I2C_ReadOneByte(DevAddr, RegAddr);
    Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
    Data <<= (8 - Length);
    Data >>= (7 - BitStart);
    Dat &= Mask;
    Dat |= Data;
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}


/**
  * @brief write an byte to I2C bus
  *         
  * @param Data: the data would be writen
  *         
  * @retval  None
**/

void I2C_WriteByte(uint8_t Data)
{
	uint8_t i;
	
	I2C_SDAMode(I2C_SDA_OUT);
	for(i = 0; i < 8; i ++)
	{
		I2C_SCL_Clr();
		Delay_us(1);
		if(Data & 0x80)
			I2C_SDA_Set();
		else
			I2C_SDA_Clr();
		Data <<= 1;
		Delay_us(1);
		I2C_SCL_Set();
		Delay_us(1);
	}
}

/**
  * @brief  Write an byte to the specified device address through I2C bus.
  *         
  * @param DevAddr: The address byte of the slave device
  * @param RegAddr: The address byte of  register of the slave device
 * @param  Data: the data would be writen to the specified device address       
  * @retval  None
**/

void I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_WriteByte(Data);
	I2C_WaiteForAck();
	I2C_Stop();
}

/**
  * @brief Write a buffer specified sizes  to the specified device address through I2C bus.
  *
  * @param DevAddr: The address byte of the slave device
  * @param RegAddr: The address byte of  register of the slave device       
  * @param Num: the sizes of the specified buffer
  * @param pBuff: point to a the specified buffer that would be writen       
  * @retval  false: paramter error
  *                true: Write a buffer succeed
**/

bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	
	for(i = 0; i < Num; i ++)
	{
		I2C_WriteByte(*(pBuff + i));
		I2C_WaiteForAck();
	}
	I2C_Stop();

	return true;
}

/**
  * @brief read an byte from I2C bus
  *         
  * @param  Ack: send an ACK/NACK to I2C bus after read an byte
  *              @arg 0: ACK
  *              @arg 1:NACK         
  * @retval  None
**/

uint8_t I2C_ReadByte(uint8_t Ack)
{
	uint8_t i, RecDat = 0;

	I2C_SDAMode(I2C_SDA_IN);
	for(i = 0; i < 8; i ++)
	{
		I2C_SCL_Clr();
		Delay_us(1);
		I2C_SCL_Set();
		RecDat <<= 1;
		if(I2C_SDA_Get())
			RecDat |= 0x01;
		else
			RecDat &= ~0x01;
		Delay_us(1);
	}
	if(I2C_ACK == Ack)
		I2C_Ack();
	else
		I2C_NAck();

	return RecDat;
}


/**
  * @brief Read an byte from the specified device address through I2C bus.
  *         
  * @param DevAddr: The address byte of the slave device
  * @param RegAddr: The address byte of  register of the slave device  
  *         
  * @retval  the byte read from I2C bus
**/

uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t TempVal = 0;
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();
	TempVal = I2C_ReadByte(I2C_NACK);
	I2C_Stop();
	
	return TempVal;
}

/**
  * @brief Read couples of bytes  from the specified device address through I2C bus.
  *
  * @param DevAddr: The address byte of the slave device
  * @param RegAddr: The address byte of  register of the slave device       
  * @param Num: the sizes of the specified buffer
  * @param pBuff: point to a the specified buffer that would read bytes from I2C bus      
  * @retval  false: paramter error
  *                true: read a buffer succeed
**/

bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();

	for(i = 0; i < Num; i ++)
	{
		if((Num - 1) == i)
		{
			*(pBuff + i) = I2C_ReadByte(I2C_NACK);
		}
		else
		{
			*(pBuff + i) = I2C_ReadByte(I2C_ACK);
		}
	}

	I2C_Stop();
	
	return true;
}

/**
  * @brief   Delay some times.
  *         
  * @param  Time
  *         
  * @retval  None
**/

void Delay_us(uint16_t Time)
{
	uint8_t i;
	
	while(Time --) for(i = 0; i < 10; i ++);
}


/**
  * @brief   Delay some times.
  *         
  * @param  Time
  *         
  * @retval  None
**/

void Delay_ms(uint16_t Time)
{
	while(Time --) Delay_us(1000);
}

/******************* (C) COPYRIGHT 2014 Waveshare *****END OF FILE*******************/

