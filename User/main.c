#include "stm32f4xx.h"
#include "led.h"
#include "I2C.h"
#include "IMU.h"
#include <stdio.h>
void Delay(__IO uint32_t nCount);  //��������

float angles[3];

int main(void)
{
  LED_GPIO_Config();                  //GPIO�ܽų�ʼ��
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
*�������ƣ�void Delay(__IO uint32_t nCount)
*
*��ڲ�������
*
*���ڲ�������
*
*����˵������ʱ����
*******************************************************************************************/
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}





