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

/* Pin Configuration
 * Board - Fira_2014
 * Current Processor Speed - 168MHz
 *
 * Pins for LEDs: PB14 - PB15
 * Pin for Switch: 	PB0-Id_1
 * 					PB1=Id_2
 * 					PB10-Id_0
 * 					PB11-Bot Color
 *
 * Pins for Motor PWM - PB6 PB7 Motor 1
 * 						PB8 PB9 Motor 2
 *
 * PWM on Timer 4
 * Timer 4 Channels - 	PB6 Channel 1
 *						PB7 Channel 2
 * 						PB8 Channel 3
 * 						PB9 Channel 4
 *
 * 	UART on Channel 2
 * 	Pins for UART2 -  	PA3 Rx
 * 						PA2 Tx
 *
 * 	Pins for UART6 -    PC7 Rx
 * 						PC6 Tx
 *
 * 	Timer Interrupt on Timer 2
 *
 *  Pins For SPI - SCK - PC10 ,  MISO - PC11 , MOSI - PC12
 *  			   CE  - PC0  ,  CSN  - PC15 , IRQ  - PD2
 *
 *
 *
 */

/* Receiver address */
uint8_t TxAddress[] = {
    0xA2,
    0xA2,
    0xA2,
    0xA2,
    0xA2
};
/* My address */
uint8_t MyAddress[] = {
    0x2A,
    0x2A,
    0x2A,
    0x2A,
    0x2A
};

#define STORE_SIZE 200
volatile int8_t VL=0,VR=0,Store[STORE_SIZE][5]={0},Send_Back_Flag=0;
volatile int NumStored = 0; // number of packets stored in Store
volatile uint8_t StoreIdx = 0,Data=0,count=0, cnt=0; // idx to store next packet

volatile uint8_t TeamID=0;
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
volatile int Usart_Flag[2]={0};

TM_NRF24L01_Transmit_Status_t transmissionStatus;

void Error_Fuzzification(int i)
{
	Positive_Error[i] = Negative_Error[i] = Zero_Error[i] = 0;
		if(Error[i]>10)
		{
			Positive_Error[i] = 10;
		}
		else if(Error[i]<-10)
		{
			Negative_Error[i] = 10;
		}
		else
		{
			if(Error[i] > 0)
			{
				Positive_Error[i] = (Error[i]);
				Zero_Error[i] = 10 - ((Error[i]));
			}
			else
			{
				Negative_Error[i] =  -((Error[i]));
				Zero_Error[i] = 10 + (Error[i]);
			}
		}
		Positive_D_Error[i] = Negative_D_Error[i] = Zero_D_Error[i] = 0;
		if(D_Error[i]>10)
		{
			Positive_D_Error[i] = 10;
		}
		else if(D_Error[i]<-10)
		{
			Negative_D_Error[i] = 10;
		}
		else
		{
			if(D_Error[i] > 0)
			{
				Positive_D_Error[i] = (D_Error[i]);
				Zero_D_Error[i] = 10 - ((D_Error[i]));
			}
			else
			{
				Negative_D_Error[i] =  -(D_Error[i]);
				Zero_D_Error[i] = 10 + (D_Error[i]);
			}
		}
}

void Create_Fuzzy_Matrix(int i)
{
		Fuzzy_Matrix[i][0][0] = Negative_D_Error[i] < Negative_Error[i] ? Negative_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][0][1] = Zero_D_Error[i] < Negative_Error[i] ? Zero_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][0][2] = Positive_D_Error[i] < Negative_Error[i] ? Positive_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][1][0] = Negative_D_Error[i] < Zero_Error[i] ? Negative_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][1][1] = Zero_D_Error[i] < Zero_Error[i] ? Zero_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][1][2] = Positive_D_Error[i] < Zero_Error[i] ? Positive_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][2][0] = Negative_D_Error[i] < Positive_Error[i] ? Negative_D_Error[i] : Positive_Error[i];
		Fuzzy_Matrix[i][2][1] = Zero_D_Error[i] < Positive_Error[i] ? Zero_D_Error[i] : Positive_Error[i];
		Fuzzy_Matrix[i][2][2] = Positive_D_Error[i] < Positive_Error[i] ? Positive_D_Error[i] : Positive_Error[i];

}
void Determine_Weights(int i)
{
	Kp_Multiplier[1] = Kp_Multiplier[0] = Kd_Multiplier[1] = Kd_Multiplier[0] = 0;

		Kp_Small[i] = Fuzzy_Matrix[i][1][1];

		Kp_Medium[i] = Fuzzy_Matrix[i][1][0] > Fuzzy_Matrix[i][0][1] ? Fuzzy_Matrix[i][1][0] : Fuzzy_Matrix[i][0][1];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][2][1] ? Kp_Medium[i] : Fuzzy_Matrix[i][2][1];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][1][2] ? Kp_Medium[i] : Fuzzy_Matrix[i][1][2];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][0][0] ? Kp_Medium[i] : Fuzzy_Matrix[i][0][0];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][2][2] ? Kp_Medium[i] : Fuzzy_Matrix[i][2][2];


		Kp_Large[i] =  Fuzzy_Matrix[i][0][2] > Fuzzy_Matrix[i][2][0] ? Fuzzy_Matrix[i][0][2] : Fuzzy_Matrix[i][2][0];


		Kd_Small[i] = Fuzzy_Matrix[i][0][2] > Fuzzy_Matrix[i][2][0] ? Fuzzy_Matrix[i][0][2] : Fuzzy_Matrix[i][2][0];
		Kd_Small[i] = Kd_Small[i] > Fuzzy_Matrix[i][1][1] ? Kd_Small[i] : Fuzzy_Matrix[i][1][1];


		Kd_Medium[i] = Fuzzy_Matrix[i][0][0] > Fuzzy_Matrix[i][0][1] ? Fuzzy_Matrix[i][0][0] : Fuzzy_Matrix[i][0][1];
		Kd_Medium[i] = Kd_Medium[i] > Fuzzy_Matrix[i][2][1] ? Kd_Medium[i] : Fuzzy_Matrix[i][2][1];
		Kd_Medium[i] = Kd_Medium[i] > Fuzzy_Matrix[i][2][2] ? Kd_Medium[i] : Fuzzy_Matrix[i][2][2];


		Kd_Large[i] = Fuzzy_Matrix[i][1][0] > Fuzzy_Matrix[i][1][2] ? Fuzzy_Matrix[i][1][0] : Fuzzy_Matrix[i][1][2];


		Kp_Multiplier[i] = (Kp_Small[i] * Kp_Small_Value) + (Kp_Medium[i] * Kp_Medium_Value) + (Kp_Large[i] * Kp_Large_Value);
		Kd_Multiplier[i] = (Kd_Small[i] * Kd_Small_Value) + (Kd_Medium[i] * Kd_Medium_Value) + (Kd_Large[i] * Kd_Large_Value);

		Kp_Divider[i] = (Kp_Small[i] + Kp_Medium[i] + Kp_Large[i]);
		Kd_Divider[i] = (Kd_Small[i] + Kd_Medium[i] + Kd_Large[i]);
}
void Give_Motor_Velocity(int i)
{
		if(Kp_Divider[i] != 0 && Kd_Divider[i] != 0)
		{
			Fault[i] = ((Error[i] * Kp_Multiplier[i]) / Kp_Divider[i]);
			Fault[i] += ((D_Error[i] * Kd_Multiplier[i]) / Kd_Divider[i]);
		}
		Motor_Velocity[i] += Fault[i];

		if(i==0)
		{
	if(Motor_Velocity[0] < 0)
	{
		if(Motor_Velocity[0] < -254)
		{

		//	TIM_SetCompare2(TIM4,254);
			Motor_Velocity[0] = -254;
		}
		TIM_SetCompare2(TIM4,254);
		TIM_SetCompare1(TIM4,254+Motor_Velocity[0]);
	}
	else
	{
		if(Motor_Velocity[0] > 254)
		{
			//TIM_SetCompare1(TIM4,254);
			Motor_Velocity[0] = 254;
		}
		TIM_SetCompare1(TIM4,254);
		TIM_SetCompare2(TIM4,254-Motor_Velocity[0]);
	}
		}

		else if(i==1)
		{
	if(Motor_Velocity[1] < 0)
	{
		if(Motor_Velocity[1] <-254)
		{
			//TIM_SetCompare3(TIM4,254);
			Motor_Velocity[1] = -254;
		}

		TIM_SetCompare3(TIM4,254);
		TIM_SetCompare4(TIM4,254+Motor_Velocity[1]);
	}
	else
	{
		if(Motor_Velocity[1] >254)
		{
			//TIM_SetCompare4(TIM4,254);
			Motor_Velocity[1] = 254;
		}
		TIM_SetCompare4(TIM4,254);
		TIM_SetCompare3(TIM4,254-Motor_Velocity[1]);
	}}
}

int main()
{
	 int i=0,j=0;
	Initialise_Clock();
	Initialise_GPIO();
	Initialise_TimerPWM();
	//Initialise_ExternalInterrupt();
	Initialise_TimerInterrupt();
	Initialise_UART2();
	Initialise_UART3();
    Initialise_UART6();

//    TM_NRF24L01_Init(15,15);
//     // Set RF settings, Data rate to 2Mbps, Output power to 0dBm
//   	 TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_2M, TM_NRF24L01_OutputPower_0dBm);
//   	 // Set my address, 5 bytes
//   	 TM_NRF24L01_SetMyAddress(MyAddress);
//   	 // Set TX address, 5 bytes
//   	 TM_NRF24L01_SetTxAddress(TxAddress);

   	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);

   	 enable_timer_int=0,count=0;

   	 //while(1);

//   		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11))
//   		{
//   			TeamID=Team_Id_2;
//   		}
//   		else
//   		{
//   			TeamID=Team_Id_1;
//   		}
   		//Bot_Id=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)*4+GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)*2+GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);

   	    TeamID = 127;
   	    Bot_Id = 1;
   	    while(1)
   		{
   			if(Cycle_Complete_Flag == 1)
   			{
   				Cycle_Complete_Flag = 0;
   				while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
   				USART_SendData(USART2,0);
   				while(USART_GetFlagStatus(USART6,USART_FLAG_TXE) == RESET);
   				USART_SendData(USART6,0);
   			}
   			for(i=0;i<2;i++)
   			{
   			if(Usart_Flag[i]==1)
   			{
   			 	Error_Fuzzification(i);
   				Create_Fuzzy_Matrix(i);
   				Determine_Weights(i);
   				Give_Motor_Velocity(i);
   				Usart_Flag[i]=0;
   			}
   			}
   		}
}

void USART3_IRQHandler()
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		Data=USART_ReceiveData(USART3);
		if((Data==TeamID) && Flag==0 )
		{
			Flag++;
		}
		else if(Flag==2*Bot_Id+1)
		{
			if(Data>127)
			{
				Velocity[0]= -2*(255-Data);
			}
			else
			{
				Velocity[0]= 2*Data;
			}
			Flag++;
		}
		else if(Flag==2*Bot_Id+2)
		{
			if(Data>127)
			{
				Velocity[1]= -2*(255-Data);
			}
			else
			{
				Velocity[1]= 2*Data;
			}
			Flag++;
		}
		else if(Flag!=0)
		{
			Flag++;
		}
		if(Flag==5)
		{
			Target[0]=Velocity[0];
			Target[1]=Velocity[1];
			Flag=0;
		}
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART3_IRQn);
	}
}

void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		Ticks[0]= USART_ReceiveData(USART2);
		if(Ticks[0]>128)
		{
			Ticks[0]=-(255-Ticks[0]);
		}
		Ticks[0]=2*Ticks[0];
		Error[0] = (Target[0] - Ticks[0]);
		D_Error[0] = Error[0] - Previous_Error[0];
		Previous_Error[0] = Error[0];
		Usart_Flag[0]=1;
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART2_IRQn);
	}
}
void USART6_IRQHandler()
{
	if(USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
	{
		Ticks[1]=  USART_ReceiveData(USART6);
		if(Ticks[1]>128)
				{
					Ticks[1]=-(255-Ticks[1]);
				}
		Ticks[1]=2*Ticks[1];
		Error[1] = (Target[1] - Ticks[1]);
		D_Error[1] = Error[1] - Previous_Error[1];
		Previous_Error[1] = Error[1];
		Usart_Flag[1]=1;
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART6_IRQn);
	}
}

/*
void EXTI2_IRQHandler()
{
	if((EXTI->IMR & EXTI_IMR_MR2) && (EXTI->PR & EXTI_PR_PR2))
	{
		GPIO_ToggleBits(GPIOB, GPIO_Pin_14);

		if (TM_NRF24L01_DataReady())
        	{
        		TM_NRF24L01_GetData(Data);
        	    TM_NRF24L01_PowerUpRx();
        	    //GPIO_SetBits(GPIOB, GPIO_Pin_15);
        	    GPIO_ToggleBits(GPIOB, GPIO_Pin_15);
        	}

//		TM_NRF24L01_Transmit(Data);
//		 do
//		 {
//		   transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
//		 } while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
//
//		 TM_NRF24L01_PowerUpRx();


		if(Data[0] == TeamID)
		{
			//GPIO_SetBits(GPIOB, GPIO_Pin_15);

			if(Data[2*Bot_Id+1]>127)
			{
				Velocity[0]= -2*(255-Data[2*Bot_Id+1]);
			}
			else
			{
				Velocity[0]= 2*Data[2*Bot_Id+1];
			}

			if(Data[2*Bot_Id+2]>127)
			{
				Velocity[1]= -2*(255-Data[2*Bot_Id+2]);
			}
			else
			{
				Velocity[1]= 2*Data[2*Bot_Id+2];
			}

			Target[0] = Velocity[0];
			Target[1] = Velocity[1];

			//GPIO_ToggleBits(GPIOB, GPIO_Pin_15);
		}
		else
		{
			//GPIO_ToggleBits(GPIOB, GPIO_Pin_14);
		}

		EXTI_ClearITPendingBit(EXTI_Line2);
		NVIC_ClearPendingIRQ(EXTI2_IRQn);
	}
}
*/

void TIM2_IRQHandler()
{
	   //GPIO_ToggleBits(GPIOB,GPIO_Pin_15);
	    Cycle_Complete_Flag = 1;
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		NVIC_ClearPendingIRQ(TIM2_IRQn);
}
