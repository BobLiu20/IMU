/******************** (C) COPYRIGHT  源地工作室 ********************************
 * 文件名  ：usart.c
 * 描述    ：将printf函数重定向到USART1。这样就可以用printf函数将单片机的数据
             打印到PC上的超级终端或串口调试助手。     
 * 作者    ：zhuoyingxingyu
 * 淘宝    ：源地工作室http://vcc-gnd.taobao.com/
 * 论坛地址：极客园地-嵌入式开发论坛http://vcc-gnd.com/
 * 版本更新: 2015-02-20
 * 硬件连接: TX--PA9;RX---PA10
 * 调试方式：J-Link-OB
**********************************************************************************/	

//头文件
#include "usart.h"

 /**
* @file   USART1_Config
* @brief  USART1/GPIO 配置,工作模式配置。9600-8-N-1
* @param  无
* @retval 无
*/
void USART1_Config(void)
{	
  GPIO_InitTypeDef GPIO_InitStructure;	
  USART_InitTypeDef USART_InitStructure;  //定义串口初始化结构
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  /*USART1_TX ->PA9*/	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; //选中串口默认输出管脚 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //定义输出最大速率 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//定义管脚9的模式  
  GPIO_Init(GPIOA, &GPIO_InitStructure);           //调用函数，把结构体参数输入进行初始化		   
  /*USART1_RX ->PA10*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
  /*串口通讯参数设置*/
  USART_InitStructure.USART_BaudRate = 9600; //波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据位8位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位1位
  USART_InitStructure.USART_Parity = USART_Parity_No;		//校验位 无
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//使能接收和发送引脚
  //将以上赋完值的结构体带入库函数USART_Init进行初始化
  USART_Init(USART1, &USART_InitStructure); 
  USART_Cmd(USART1, ENABLE);//开启USART1，注意与上面RCC_APB2PeriphClockCmd()设置的区别
}




