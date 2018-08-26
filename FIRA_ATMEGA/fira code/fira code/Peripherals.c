#include <avr/io.h>
#include <avr/interrupt.h>

#include "Peripherals.h"

void Initialise_Interrupt(void)
{
	EICRA = 0b00000010;
	EIMSK = 0b00000001;				//Configure INT0(PD2) and INT1(PD3) as falling edge triggered interrupts //Enable INT0 & INT1
	return;
}
void Initialise_Port(void)
{
	DDRD = 0b00000010;
	PORTD |= 0b00000101;
	
	DDRC = 255;
	PORTC = 0;
}
void Initialise_UART(uint16_t ubrrvalue)
{
	UBRR0H=(unsigned char)(ubrrvalue>>8);
	UBRR0L=(unsigned char)ubrrvalue;

	UCSR0C = (3<<UCSZ00);

	UCSR0B = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
	
}
void Initialise_Timer(void)
{
	TCCR2A = 0b00000010;			//Timer2 configured to generate Interrupts at every 3.2ms interval
	TCCR2B = 0b00000011;			//CTC mode

	OCR2A = 100;
	Set_TimerInt;
}