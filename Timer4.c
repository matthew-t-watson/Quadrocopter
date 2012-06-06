#include <stdio.h>
#include <p33Fj128GP202.h>
#include "C:\Users\Matt\Quadrocopter\common.h"

void Setup_Timer4()
{
	IEC1bits.T4IE = 0; //Disable timer4 interrupt
	T4CONbits.TON = 0; //Disable timer
	T4CONbits.TSIDL = 0; //Continue operation in idle mode
	T4CONbits.TGATE = 0; //Timer gate accumulation disabled
	T4CONbits.TCKPS = 0b11; //Timer prescale 1:1, 1:8, 1:64, 1:256
	T4CONbits.T32 = 0; //32 bit timer disabled
	T4CONbits.TCS = 0; //Internal clock source
	
	//Resets after 0.04s
	PR4 = 6250; //Period register
	
	TMR4=0;
	T4CONbits.TON = 1; //Enable timer4
	IEC1bits.T4IE = 1; //Enable timer4 interrupt
	printf("\nTimer4 Setup Complete");
}

void Reset_Timer4()
{
	TMR4=0;
}	

void _ISR _T4Interrupt(void)
{	
	IFS1bits.T4IF = 0; //Clear interrupt flag
	IEC1bits.T4IE = 0; //Disable timer4 interrupt
	
	//Sets all motors to their stationary value. The watchdog reset will only occur if the reciever recieves no data for a certain period, aka if the signal is lost. This does mean the quad will drop out of the sky, but its better than it flying away and hitting something/someone with blades still spinning.
	throttle = 0.0;
	OC1R = 700;
	OC2R = 700;
	OC3R = 700;
	OC4R = 700;
	output_compare_fire();
	printf("\nSignal Lost");
	
	TMR4=0;
	IEC1bits.T4IE = 1; //Enable timer1 interrupt
}

/*//dt measuring timer (technically not required with fixed 400Hz loop)
void Setup_Timer4()
{
	IEC1bits.T4IE = 0; //Disable timer4 interrupt
	T4CONbits.TON = 0; //Disable timer
	T4CONbits.TSIDL = 0; //Continue operation in idle mode
	T4CONbits.TGATE = 0; //Timer gate accumulation disabled
	T4CONbits.TCKPS = 0b10; //Timer prescale 1:1, 1:8, 1:64, 1:256
	T4CONbits.T32 = 0; //32 bit timer disabled
	T4CONbits.TCS = 0; //Internal clock source
	
	//Will be manually reset
	PR4 = 65535; //Period register
	
	TMR4=0;
	T4CONbits.TON = 1; //Enable timer
	printf("\nTimer4 Setup Complete");
}

void Get_dt()
{
	dt = TMR4 * 0.0000016156; // 0.0000016
	TMR4 = 0;
}*/