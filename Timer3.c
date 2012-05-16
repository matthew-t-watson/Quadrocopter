#include <stdio.h>
#include <p33Fj128GP202.h>
#include "D:\documents\Matthew\mplab\ControlV4\common.h"

//Output pulse timer
void Setup_Timer3()
{
	IEC0bits.T3IE = 0; //Disable timer2 interrupt
	T3CONbits.TON = 0; //Disable timer
	T3CONbits.TSIDL = 0; //Continue operation in idle mode
	T3CONbits.TGATE = 0; //Timer gate accumulation disabled
	T3CONbits.TCKPS = 0b01; //Timer prescale 1:1, 1:8, 1:63, 1:256
	//T3CONbits.T32 = 0; //32 bit timer disabled
	T3CONbits.TCS = 0; //Internal clock source
	

	PR3 = 65535; //Period register
	
	TMR3=0;	
	printf("\nTimer3 Setup Complete");
}


void enable_timer3()
{
	TMR3=0;
	T3CONbits.TON = 1;
}

void disable_timer3()
{
	TMR3=0;
	T3CONbits.TON = 0;
}