/*
 * AtmegaNRF.c
 *
 * Created: 6/27/2015 10:57:40 PM
 *  Author: Adarsh Kosta
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "NRF24L01.h"

#define baud 38400
#define PLOAD 11

extern volatile uint8_t TX_BUF[];
extern volatile uint8_t RX_BUF[];
volatile uint8_t send_cnt = 0, rec_cnt = 0;
volatile int data_flag=0,irq_flag=0;

void Initialise_UART(uint16_t ubrrvalue)
{
	DDRD |=(1<<DDD1);
	DDRD &= ~(1<<DDD0);
	PORTD |= (1<<DDD0)|(1<<DDD1);
	UBRR0H=(unsigned char)(ubrrvalue>>8);
	UBRR0L=(unsigned char)ubrrvalue;

	UCSR0C = (3<<UCSZ00);

	UCSR0B = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
}

void Uart_Write(uint8_t *Data)
{
	for(rec_cnt=0; rec_cnt<PLOAD; rec_cnt++)
	{	
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0=Data[rec_cnt];
	}
	rec_cnt = 0;		
}

ISR(USART_RX_vect)
{
	TX_BUF[send_cnt]=UDR0;
	send_cnt++;
	if(send_cnt == PLOAD)
	{
		data_flag=1;
	    send_cnt = 0;
	}		
}
ISR(INT0_vect)
{
	irq_flag=1;
}
void init_exti()
{
	DDRD &= ~(1<<DDD2);
	EICRA|=(1<<ISC01);
	EIMSK|=(1<<INT0);
}
int main(void)
{
	nRF24L01_Initial();
	nRF24L01_Config();
	init_exti();
	Initialise_UART(25);
	sei();
	while(1)
	{
		if(data_flag==1)
		{
			NRF24L01_Send();
			data_flag=0;
		}
		if(irq_flag==1)
		{
			NRF24L01_Receive();
			Uart_Write(RX_BUF);
			irq_flag=0;
		}
		
	}
}