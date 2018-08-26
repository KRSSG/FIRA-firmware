#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_


void Initialise_Clock();
void Initialise_UART();
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

#define BotMaxVelocity 120
#define BotMinVelocity -120
#define Team_Id_1  126
#define Team_Id_2  127
#define HIGH 250




#endif /* PERIPHERALS_H_ */
