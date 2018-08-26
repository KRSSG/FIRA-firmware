


#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#define Clear_TimerInt TIMSK2 &= (~(1 << OCIE2A));
#define Set_TimerInt TIMSK2 |= ((1 << OCIE2A));

void Initialise_Interrupt(void);
void Initialise_Timer(void);
void Initialise_Port(void);
void Initialise_ADC();
void Initialise_UART(uint16_t ubrrvalue);



#endif /* PERIPHERALS_H_ */