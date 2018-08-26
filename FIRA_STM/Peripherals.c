#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_syscfg.h>
#include <stm32f4xx_flash.h>

#include "Peripherals.h"

void Initialise_Clock()
{
	/*!------------------------------------------- GPIO -------------------------------------------!*/

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);

	/*!------------------------------------------- For GPIO -------------------------------------------!*/

	/*!------------------------------------ For External Interrupt ------------------------------------!*/

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*!------------------------------------ For External Interrupt ------------------------------------!*/

	/*!-------------------------------------------- For UART ------------------------------------------!*/

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	/*!-------------------------------------------- For UART ------------------------------------------!*/

	/*!-------------------------------------------- For Timer ------------------------------------------!*/

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //For PWM
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //For Timer Interrupt

	/*!-------------------------------------------- For Timer ------------------------------------------!*/
}


void Initialise_UART2()
{
	/*!--------------------------------------- Initialise Structure ----------------------------------------!*/

	USART_InitTypeDef UART2_DEF;

	UART2_DEF.USART_BaudRate = 38400;
	UART2_DEF.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART2_DEF.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	UART2_DEF.USART_Parity = USART_Parity_No;
	UART2_DEF.USART_StopBits = USART_StopBits_1;
	UART2_DEF.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART2,&UART2_DEF);

	/*!---------------------------------------- Initialise Structure ----------------------------------------!*/

	/*!---------------------------------------- Interrupt Controller ----------------------------------------!*/

	NVIC_InitTypeDef NVIC_UART;

	NVIC_UART.NVIC_IRQChannel =  USART2_IRQn;
	NVIC_UART.NVIC_IRQChannelCmd = ENABLE;
	NVIC_UART.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_UART.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&NVIC_UART);

	/*!---------------------------------------- Interrupt Controller ----------------------------------------!*/

	/*!---------------------------------------- Enable UART Interrupt ----------------------------------------!*/

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

	/*!---------------------------------------- Enable UART Interrupt ----------------------------------------!*/

	/*!---------------------------------------- Enable UART  ----------------------------------------!*/

	USART_Cmd(USART2,ENABLE);

	/*!---------------------------------------- Enable UART  ----------------------------------------!*/
}

void Initialise_UART6()
{
	/*!--------------------------------------- Initialise Structure ----------------------------------------!*/

	USART_InitTypeDef UART6_DEF;

	UART6_DEF.USART_BaudRate = 38400;
	UART6_DEF.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART6_DEF.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	UART6_DEF.USART_Parity = USART_Parity_No;
	UART6_DEF.USART_StopBits = USART_StopBits_1;
	UART6_DEF.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART6,&UART6_DEF);

	/*!---------------------------------------- Initialise Structure ----------------------------------------!*/

	/*!---------------------------------------- Interrupt Controller ----------------------------------------!*/

	NVIC_InitTypeDef NVIC_UART;

	NVIC_UART.NVIC_IRQChannel =  USART6_IRQn;
	NVIC_UART.NVIC_IRQChannelCmd = ENABLE;
	NVIC_UART.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_UART.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&NVIC_UART);

	/*!---------------------------------------- Interrupt Controller ----------------------------------------!*/

	/*!---------------------------------------- Enable UART Interrupt ----------------------------------------!*/

	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);

	/*!---------------------------------------- Enable UART Interrupt ----------------------------------------!*/

	/*!---------------------------------------- Enable UART  ----------------------------------------!*/

	USART_Cmd(USART6,ENABLE);

	/*!---------------------------------------- Enable UART  ----------------------------------------!*/
}

void Initialise_UART3()
{
	/*!------------------------------------------- For UART3 -------------------------------------------!*/

	GPIO_InitTypeDef GPIO_UART3;

	GPIO_UART3.GPIO_Mode = GPIO_Mode_AF;
	GPIO_UART3.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_UART3.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_UART3.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_UART3.GPIO_OType = GPIO_OType_PP;

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11, GPIO_AF_USART3);

	GPIO_Init(GPIOC,&GPIO_UART3);

	/*!------------------------------------------- For UART3 ----------------------------------------------!*/
	/*!--------------------------------------- Initialise Structure ----------------------------------------!*/

	USART_InitTypeDef UART3_DEF;

	UART3_DEF.USART_BaudRate = 38400;
	UART3_DEF.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	UART3_DEF.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	UART3_DEF.USART_Parity = USART_Parity_No;
	UART3_DEF.USART_StopBits = USART_StopBits_1;
	UART3_DEF.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART3,&UART3_DEF);

	/*!---------------------------------------- Initialise Structure ----------------------------------------!*/

	/*!---------------------------------------- Interrupt Controller ----------------------------------------!*/

	NVIC_InitTypeDef NVIC_UART;

	NVIC_UART.NVIC_IRQChannel =  USART3_IRQn;
	NVIC_UART.NVIC_IRQChannelCmd = ENABLE;
	NVIC_UART.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_UART.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_UART);

	/*!---------------------------------------- Interrupt Controller ----------------------------------------!*/

	/*!---------------------------------------- Enable UART Interrupt ----------------------------------------!*/

	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);

	/*!---------------------------------------- Enable UART Interrupt ----------------------------------------!*/

	/*!---------------------------------------- Enable UART  ----------------------------------------!*/

	USART_Cmd(USART3,ENABLE);

	/*!---------------------------------------- Enable UART  ----------------------------------------!*/
}

void Initialise_GPIO()

{
	/*!------------------------------------------- For LEDs -------------------------------------------!*/

	GPIO_InitTypeDef GPIO_LED;

	GPIO_LED.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_LED.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_LED.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_LED.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_LED.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOB,&GPIO_LED);

	/*!------------------------------------------- For LEDs -------------------------------------------!*/

	/*!--------------------------------------------For Switch------------------------------------------!*/

	GPIO_InitTypeDef GPIO_SWITCH;

	GPIO_SWITCH.GPIO_Mode = GPIO_Mode_IN;
	GPIO_SWITCH.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_SWITCH.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_SWITCH.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_SWITCH.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOB,&GPIO_SWITCH);

	/*!--------------------------------------------For Switch------------------------------------------!*/


	/*!------------------------------------------- For PWM -------------------------------------------!*/

	GPIO_InitTypeDef GPIO_PWM;

	GPIO_PWM.GPIO_Mode = GPIO_Mode_AF;
	GPIO_PWM.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_PWM.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_PWM.GPIO_OType = GPIO_OType_PP;
	GPIO_PWM.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9, GPIO_AF_TIM4);

	GPIO_Init(GPIOB,&GPIO_PWM);

	/*!------------------------------------------- For PWM -------------------------------------------!*/

	/*!------------------------------------------- For UART2 -------------------------------------------!*/

	GPIO_InitTypeDef GPIO_UART2;

	GPIO_UART2.GPIO_Mode = GPIO_Mode_AF;
	GPIO_UART2.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_UART2.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_UART2.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_UART2.GPIO_OType = GPIO_OType_PP;

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2);

	GPIO_Init(GPIOA,&GPIO_UART2);

	/*!------------------------------------------- For UART2 -------------------------------------------!*/

	/*!------------------------------------------- For UART6 -------------------------------------------!*/

	GPIO_InitTypeDef GPIO_UART6;

	GPIO_UART6.GPIO_Mode = GPIO_Mode_AF;
	GPIO_UART6.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_UART6.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_UART6.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_UART6.GPIO_OType = GPIO_OType_PP;

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_Init(GPIOC,&GPIO_UART6);

	/*!------------------------------------------- For UART6 -------------------------------------------!*/

	/*!------------------------------------ For External Interrupt ---------------------------------------!*/

	GPIO_InitTypeDef GPIO_IRQ;

	GPIO_IRQ.GPIO_Mode = GPIO_Mode_IN;
	GPIO_IRQ.GPIO_Pin = GPIO_Pin_2;
	GPIO_IRQ.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_IRQ.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_IRQ.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOD,&GPIO_IRQ);

	/*!------------------------------------ For External Interrupt ---------------------------------------!*/

}

void Initialise_TimerInterrupt()
{
	/*!----------------------------- Initialise Base Structure ---------------------------------!*/

	TIM_TimeBaseInitTypeDef TIM2_INIT;

	TIM2_INIT.TIM_Prescaler =8400-1;
	TIM2_INIT.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM2_INIT.TIM_CounterMode = TIM_CounterMode_Up;
	TIM2_INIT.TIM_Period = 32;

	TIM_TimeBaseInit(TIM2,&TIM2_INIT);

	/*!----------------------------- Initialise Base Structure ---------------------------------!*/

	/*!----------------------------- Interrupt Controller ---------------------------------!*/

	NVIC_InitTypeDef NVIC_TIMER;

	NVIC_TIMER.NVIC_IRQChannel =  TIM2_IRQn;
	NVIC_TIMER.NVIC_IRQChannelCmd = ENABLE;
	NVIC_TIMER.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_TIMER.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&NVIC_TIMER);

	/*!----------------------------- Interrupt Controller ---------------------------------!*/

	/*!----------------------------- Enable Interrupt ---------------------------------!*/

	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);

	/*!----------------------------- Enable Interrupt ---------------------------------!*/

	/*!----------------------------- Enable Timer ---------------------------------!*/

	TIM_Cmd(TIM2,ENABLE);

	/*!----------------------------- Enable Timer ---------------------------------!*/
}

void Initialise_ExternalInterrupt()
{
	/*!------------------------------------ Initialise Structures ---------------------------------------!*/

	EXTI_InitTypeDef EXTI_IRQ;

	EXTI_IRQ.EXTI_Line = EXTI_Line2;
	EXTI_IRQ.EXTI_LineCmd = ENABLE;
	EXTI_IRQ.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_IRQ.EXTI_Trigger = EXTI_Trigger_Falling;

	EXTI_Init(&EXTI_IRQ);

	/*!------------------------------------ Initialise Structures ---------------------------------------!*/

	/*!------------------------------------ Configure Lines ---------------------------------------!*/

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource2);

	/*!------------------------------------ Configure Lines ---------------------------------------!*/

	/*!---------------------------------- Interrupt Controller ---------------------------------------!*/

	NVIC_InitTypeDef NVIC_IRQ;

	NVIC_IRQ.NVIC_IRQChannel =  EXTI2_IRQn ;
	NVIC_IRQ.NVIC_IRQChannelCmd = ENABLE;
	NVIC_IRQ.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_IRQ.NVIC_IRQChannelSubPriority = 1;

	NVIC_Init(&NVIC_IRQ);
	NVIC_EnableIRQ(EXTI2_IRQn );

	/*!---------------------------------- Interrupt Controller ---------------------------------------!*/
}

void Initialise_TimerPWM()
{
	/*!----------------------------- Initialise Base Structures -----------------------------!*/

	TIM_TimeBaseInitTypeDef TIM4_INIT;
	TIM_OCInitTypeDef TIM4_OC;

	TIM4_INIT.TIM_Prescaler = 100-1;
	TIM4_INIT.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM4_INIT.TIM_CounterMode = TIM_CounterMode_Up;
	TIM4_INIT.TIM_Period = 255; //8 bit equivalent

	TIM_TimeBaseInit(TIM4,&TIM4_INIT);

	/*!----------------------------- Initialise Base Structures -----------------------------!*/

	/*!----------------------------- OC Mode -----------------------------!*/

	TIM4_OC.TIM_OCMode = TIM_OCMode_PWM1;
	TIM4_OC.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM4_OC.TIM_OutputState = TIM_OutputState_Enable ;

	TIM_OC1Init(TIM4,&TIM4_OC);
	TIM_OC2Init(TIM4,&TIM4_OC);
	TIM_OC3Init(TIM4,&TIM4_OC);
	TIM_OC4Init(TIM4,&TIM4_OC);

	/*!----------------------------- OC Mode -----------------------------!*/

	/*!----------------------------- Enable Timer -----------------------------!*/

	TIM_Cmd(TIM4, ENABLE);
	TIM_SetCompare1(TIM4, 0);
	TIM_SetCompare2(TIM4, 0);
	TIM_SetCompare3(TIM4, 0);
	TIM_SetCompare4(TIM4, 0);

	/*!----------------------------- Enable Timer -----------------------------!*/
}


void delay(int i)
{
	long time;
	time = 100000*i;
	for( ; time > 0; time -- )
	{

	}
}
