//ͷ�ļ�
#include "led.h"

//=============================================================================
//��������:LED_GPIO_Config(void)
//���ܸ�Ҫ:LED����������
//��������:��
//��������:��
//=============================================================================
void LED_GPIO_Config(void)
{	
	//����һ��GPIO_InitTypeDef ���͵Ľṹ��
	GPIO_InitTypeDef  GPIO_InitStructure;
	//ʹ��GPIOC������ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//ѡ��Ҫ�õ�GPIO����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	//��������Ϊ��ͨ���ģʽ				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//��������Ϊ�������
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//���������ٶ�Ϊ100MHz
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	//��������Ϊ����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 		 
	//���ÿ⺯������ʼ��GPIO
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}






