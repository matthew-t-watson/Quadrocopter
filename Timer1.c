#include <p33Fj128GP202.h>
#include "C:\Users\Matt\Quadrocopter\common.h"

//400Hz control loop timer
void Setup_Timer1()
{
	IEC0bits.T1IE = 0; //Disable timer1 interrupt
	T1CONbits.TON = 0; //Disable timer
	T1CONbits.TSIDL = 0; //Continue operation in idle mode
	T1CONbits.TGATE = 0; //Timer gate accumulation disabled
	T1CONbits.TCKPS = 0b01; //Timer prescale 1:1, 1:8, 1:64, 1:256
	//T1CONbits.T32 = 0; //32 bit timer disabled
	T1CONbits.TCS = 0; //Internal clock source
	IPC0bits.T1IP = 0b100; //Priority 4
	
	//Frequency of 400Hz
	PR1 = 12500; //Period register
	
	TMR1=0;
	T1CONbits.TON = 1; //Enable timer
	
	printf("\nTimer1 Setup Complete");
}	

void _ISR _T1Interrupt(void)
{	
	IFS0bits.T1IF = 0; //Clear interrupt flag
	IEC0bits.T1IE = 0; //Disable timer1 interrupt
	LATAbits.LATA0 = !LATAbits.LATA0;
	
	Get_dt();
	Get_Gyro_Rates();
	Get_Accel_Values();
	Get_Accel_Angles();		
	
	//GYRO_XANGLE += GYRO_XRATE*dt;
	//GYRO_YANGLE += GYRO_YRATE*dt;
	GYRO_ZANGLE += GYRO_ZRATE*dt;		
	
	complementary_filter();
	//second_order_complementary_filter();
	
	update_PID();
	update_motors_single_shot();
	IEC0bits.T1IE = 1; //Enable timer1 interrupt
}
