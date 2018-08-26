#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_


void Initialise_Clock();
void Initialise_UART2();
void Initialise_UART6();
void Initialise_GPIO();
void Initialise_TimerInterrupt();
void Initialise_ExternalInterrupt();
void Initialise_TimerPWM();
void delay(int );

#define Kp_Small_Value 1
#define Kp_Medium_Value 2
#define Kp_Large_Value 3
#define Kd_Small_Value 3
#define Kd_Medium_Value 6
#define Kd_Large_Value 9

#define BotMaxVelocity 125
#define BotMinVelocity -125
#define Team_Id_1  126
#define Team_Id_2  127

#define HIGH 250




#endif /* PERIPHERALS_H_ */
