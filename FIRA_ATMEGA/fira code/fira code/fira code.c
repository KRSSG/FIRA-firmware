#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "Peripherals.h"
volatile int8_t Data = 0,Flag=0,Send_Ticks=0;
volatile int Ticks=0;
void Uart_Write(uint8_t Data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0=Data;
}
int main(void)
{
	Initialise_Port();
	Initialise_UART(25);	
	//Initialise_Timer();
	Initialise_Interrupt();
	sei();
	while (1)
	{
		if(Flag==1)
		{
			if(Ticks>255)
			{
				Ticks=255;
			}
			else if(Ticks<-255)
			{
				Ticks=-255;
			}
			Send_Ticks=Ticks/2;
			Uart_Write(Send_Ticks);
			Ticks=0;
			Flag=0;
		}
	}
}
/*
ISR(TIMER2_COMPA_vect)
{
	Flag=1;
	PORTC=~PORTC;
}*/
ISR(INT0_vect)
{	
	if((PIND >> 3) & 1)
	Ticks++;											
	else
	Ticks--;
}
ISR(USART_RX_vect)
{
	Data = UDR0;
	Flag=1;
}