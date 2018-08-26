
#define  F_CPU 20000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <stdlib.h>
#include "Peripherals.h"
volatile int8_t Data = 0,Flag=0;
volatile int32_t Ticks=0;
int main(void)
{
	wdt_disable();
	Initialise_Port();
	Initialise_UART(64);
	//Initialise_Timer();
	Initialise_Interrupt();
	while (1)
	{
		if(Flag)
		{
			UDR0=Ticks;
			Ticks=0;
			Flag=0;
		}
	}
}
ISR(INT1_vect)
{
	if(((PIND >> 4) & 1) ^ 1)
	Ticks++;									//M2
	else
	Ticks--;
}
ISR(USART_RX_vect)
{
	Data = UDR0;
	Flag=1;
}
