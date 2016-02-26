/******************** (C) COPYRIGHT  Դ�ع����� ********************************
 * �ļ���  ��usart.c
 * ����    ����printf�����ض���USART1�������Ϳ�����printf��������Ƭ��������
             ��ӡ��PC�ϵĳ����ն˻򴮿ڵ������֡�     
 * ����    ��zhuoyingxingyu
 * �Ա�    ��Դ�ع�����http://vcc-gnd.taobao.com/
 * ��̳��ַ������԰��-Ƕ��ʽ������̳http://vcc-gnd.com/
 * �汾����: 2015-02-20
 * Ӳ������: TX--PA9;RX---PA10
 * ���Է�ʽ��J-Link-OB
**********************************************************************************/	

//ͷ�ļ�
#include "usart.h"

 /**
* @file   USART1_Config
* @brief  USART1/GPIO ����,����ģʽ���á�9600-8-N-1
* @param  ��
* @retval ��
*/
void USART1_Config(void)
{	
  GPIO_InitTypeDef GPIO_InitStructure;	
  USART_InitTypeDef USART_InitStructure;  //���崮�ڳ�ʼ���ṹ
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  /*USART1_TX ->PA9*/	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; //ѡ�д���Ĭ������ܽ� 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //�������������� 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//����ܽ�9��ģʽ  
  GPIO_Init(GPIOA, &GPIO_InitStructure);           //���ú������ѽṹ�����������г�ʼ��		   
  /*USART1_RX ->PA10*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
  /*����ͨѶ��������*/
  USART_InitStructure.USART_BaudRate = 9600; //������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; //����λ8λ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;	//ֹͣλ1λ
  USART_InitStructure.USART_Parity = USART_Parity_No;		//У��λ ��
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//ʹ�ܽ��պͷ�������
  //�����ϸ���ֵ�Ľṹ�����⺯��USART_Init���г�ʼ��
  USART_Init(USART1, &USART_InitStructure); 
  USART_Cmd(USART1, ENABLE);//����USART1��ע��������RCC_APB2PeriphClockCmd()���õ�����
}




