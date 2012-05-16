#include <stdio.h>
#include <p33Fj128GP202.h>
#include "D:\documents\Matthew\mplab\ControlV4\common.h"

//dt measuring timer
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
}