#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_syscfg.h>
#include <stm32f4xx_flash.h>
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_nrf24l01.h"
#include "tm_stm32f4_spi.h"
#include "Peripherals.h"

#define STORE_SIZE 200
volatile int8_t VL=0,VR=0,Store[STORE_SIZE][5]={0},Send_Back_Flag=0;
volatile int NumStored = 0; // number of packets stored in Store
volatile int StoreIdx = 0; // idx to store next packet

volatile int8_t Data=0,TeamID=0;
int Bot_Id=0,Bot_Flag=0;
volatile int i=0,loop=0,p=0,j=0,enable_timer_int=0;
volatile int Previous_Error[2] = {0}, Ticks[2] = {0}, D_Error[2] = {0}, Error[2] = {0}, Target[2] = {0};
volatile int Cycle_Complete_Flag = 0, Flag = 0,Count=0,Enable_Flag=0;
volatile int Velocity[2]={0};
volatile int Motor_Velocity[2] = {0},
	Negative_Error[2] = {0}, Zero_Error[2] = {0}, Positive_Error[2] = {0},
	Negative_D_Error[2] = {0}, Zero_D_Error[2] = {0}, Positive_D_Error[2] = {0},
	Kp_Small[2] = {0}, Kp_Medium[2] = {0}, Kp_Large[2] = {0},
	Kd_Small[2] = {0}, Kd_Medium[2] = {0}, Kd_Large[2] = {0},
	Fuzzy_Matrix[2][3][3] = {{{0}}};
volatile int Kp_Multiplier[2], Kd_Multiplier[2],
	Kp_Divider[2], Kd_Divider[2],
    Fault[2] = {0};
volatile int Ticks_Array_L[100]={0},Ticks_Array_R[100]={0};

int main()
{
	int i=0,j=0;
	Initialise_Clock();
	Initialise_GPIO();
	Initialise_TimerPWM();
	Initialise_UART2();
	Initialise_UART6();

	GPIO_SetBits(GPIOB,GPIO_Pin_15);
	while(1)
	{
		TIM_SetCompare1(TIM4,235);
		TIM_SetCompare2(TIM4,255);
		TIM_SetCompare3(TIM4,235);
		TIM_SetCompare4(TIM4,255);
	}
}
void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{

		//GPIO_ToggleBits(GPIOB,GPIO_Pin_15);
		Ticks[0]=  USART_ReceiveData(USART2);
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART2_IRQn);
	}
}
void USART6_IRQHandler()
{
	if(USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
	{
		//GPIO_ToggleBits(GPIOB,GPIO_Pin_15);
		Ticks[1]=  USART_ReceiveData(USART6);
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART6_IRQn);
	}
}
