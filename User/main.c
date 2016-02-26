#include "stm32f4xx.h"
#include "led.h"
#include "I2C.h"
#include "IMU.h"
#include <stdio.h>
void Delay(__IO uint32_t nCount);  //函数声明

float angles[3];

int main(void)
{
  LED_GPIO_Config();                  //GPIO管脚初始化
	I2Cx_Init();
	IMU_Init();
  while (1)
  {
		LED1_ON;
		Delay(0XFFFFFF);
		LED1_OFF;
		Delay(0XFFFFFF);
		IMU_GetYawPitchRoll(angles);
  }
}
/********************************************************************************************
*函数名称：void Delay(__IO uint32_t nCount)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：延时函数
*******************************************************************************************/
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}





