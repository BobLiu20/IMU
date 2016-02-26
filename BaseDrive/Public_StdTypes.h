#ifndef _PUBILC_STDTYPES_H_
#define _PUBILC_STDTYPES_H_

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Public_StdMacros.h"

#define NULL     0  

typedef struct BYTE_BIT
{
    unsigned BIT0:1;
    unsigned BIT1:1;
    unsigned BIT2:1;
    unsigned BIT3:1;
    unsigned BIT4:1;
    unsigned BIT5:1;
    unsigned BIT6:1;
    unsigned BIT7:1;
}BYTEBIT;

typedef struct WORD_BIT
{
    unsigned BIT0:1;
    unsigned BIT1:1;
    unsigned BIT2:1;
    unsigned BIT3:1;
    unsigned BIT4:1;
    unsigned BIT5:1;
    unsigned BIT6:1;
    unsigned BIT7:1;
    
    unsigned BIT8:1;
    unsigned BIT9:1;
    unsigned BIT10:1;
    unsigned BIT11:1;
    unsigned BIT12:1;
    unsigned BIT13:1;
    unsigned BIT14:1;
    unsigned BIT15:1;
}WORDBIT;

typedef union
{
	uint16_t Word;
	WORDBIT  Bit;
}Bit16_OperTypeDef;

typedef struct 
{
	uint8_t Count;
	GPIO_TypeDef *GPIOx;
	GPIO_InitTypeDef GPIO_InitType;
}GPIO_TypeStruct;

typedef struct
{
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
	BitAction (*GPIO_BitVal)(void);
}LED_TypeDef;

typedef struct 
{
	uint8_t Count;
	SPI_TypeDef* SPIx;
	SPI_InitTypeDef SPI_InitStruct;
	uint32_t RCC_APBPeriph;
	void (*SPIx_PeriphClockCmd)(uint32_t RCC_SPIPeriph, FunctionalState NewState);
	void (*SPIx_GPIOInit)(void);
}SPI_TypeStruct;


#endif

/******************* (C) COPYRIGHT 2014 Waveshare *****END OF FILE*******************/

