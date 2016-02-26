//头文件
#include "led.h"

//=============================================================================
//函数名称:LED_GPIO_Config(void)
//功能概要:LED灯引脚配置
//参数名称:无
//函数返回:无
//=============================================================================
void LED_GPIO_Config(void)
{	
	//定义一个GPIO_InitTypeDef 类型的结构体
	GPIO_InitTypeDef  GPIO_InitStructure;
	//使能GPIOC的外设时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//选择要用的GPIO引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	//设置引脚为普通输出模式				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//设置引脚为推挽输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//设置引脚速度为100MHz
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	//设置引脚为上拉
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 		 
	//调用库函数，初始化GPIO
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}






