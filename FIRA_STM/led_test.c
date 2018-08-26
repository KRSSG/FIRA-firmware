#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include "Peripherals.h"

int main()
{
	Initialise_Clock();
	Initialise_GPIO();
	Initialise_TimerPWM();

	TIM_SetCompare1(TIM4,150);
	TIM_SetCompare3(TIM4,150);

	GPIO_SetBits(GPIOB,GPIO_Pin_15);

	while(1)
	{
		//GPIO_SetBits(GPIOB,GPIO_Pin_15);
		//delay(1);
	}
}
