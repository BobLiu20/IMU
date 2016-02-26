#ifndef _GPIO_H_
#define _GIPO_H_
#include "stm32f4xx.h"
#define  LED1_ON	  	GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define	 LED1_OFF	    GPIO_ResetBits(GPIOB,GPIO_Pin_9)		 
//#define	 LED2_ON	    GPIO_SetBits(GPIOC,GPIO_Pin_13)
//#define	 LED2_OFF	    GPIO_ResetBits(GPIOC,GPIO_Pin_13)	
//#define	 LED3_ON	    GPIO_SetBits(GPIOC,GPIO_Pin_2)
//#define	 LED3_OFF	    GPIO_ResetBits(GPIOC,GPIO_Pin_2)	

void LED_GPIO_Config(void);

#endif
