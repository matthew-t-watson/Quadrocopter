#include <stdio.h>
#include <p33Fj128GP202.h>

#define  PPSUnlock                   __builtin_write_OSCCONL(OSCCON & 0xbf) 
#define  PPSLock                     __builtin_write_OSCCONL(OSCCON | 0x40)

void Setup_OC1_PWM()
{
	T2CONbits.TON = 0; //Disable timer
	
	PPSUnlock;
	RPOR6bits.RP12R = 0b10010; //Tie OC1 to RP12
	RPOR6bits.RP13R = 0b10011; //Tie OC2 to RP13
	RPOR7bits.RP14R = 0b10100; //Tie OC3 to RP14
	RPOR7bits.RP15R = 0b10101; //Tie OC4 to RP15
	PPSLock;
	
	IEC0bits.OC1IE = 0; //Disable OC1 interrupt
	IEC0bits.OC2IE = 0; //Disable OC2 interrupt
	IEC1bits.OC3IE = 0; //Disable OC3 interrupt
	IEC1bits.OC4IE = 0; //Disable OC4 interrupt
	
	TRISBbits.TRISB12 = 0;
	TRISBbits.TRISB13 = 0;	
	TRISBbits.TRISB14 = 0;	
	TRISBbits.TRISB15 = 0;	
	
	OC1CONbits.OCSIDL = 0; //OC continues in idle mode
	OC1CONbits.OCFLT = 0; //Fault status bit, N/A as fault pin is disabled
	OC1CONbits.OCTSEL = 0; //Timer2 source
	OC1CONbits.OCM = 0b110; //PWM mode, no fault pin
	
	OC2CONbits.OCSIDL = 0; //OC continues in idle mode
	OC2CONbits.OCFLT = 0; //Fault status bit, N/A as fault pin is disabled
	OC2CONbits.OCTSEL = 0; //Timer2 source
	OC2CONbits.OCM = 0b110; //PWM mode, no fault pin
	
	OC3CONbits.OCSIDL = 0; //OC continues in idle mode
	OC3CONbits.OCFLT = 0; //Fault status bit, N/A as fault pin is disabled
	OC3CONbits.OCTSEL = 0; //Timer2 source
	OC3CONbits.OCM = 0b110; //PWM mode, no fault pin
	
	OC4CONbits.OCSIDL = 0; //OC continues in idle mode
	OC4CONbits.OCFLT = 0; //Fault status bit, N/A as fault pin is disabled
	OC4CONbits.OCTSEL = 0; //Timer2 source
	OC4CONbits.OCM = 0b110; //PWM mode, no fault pin
	
	
	OC1RS = 700;
	OC2RS = 700;
	OC3RS = 700;
	OC4RS = 700;
	
	TMR2=0;
	T2CONbits.TON = 1; //Enable timer
	
	printf("\nPWM Setup Complete");
	
}

	/*RPOR6bits.RP13R = 0b10011; //Tie OC2 to RP13
	RPOR7bits.RP14R = 0b10100; //Tie OC3 to RP14
	RPOR7bits.RP15R = 0b10101; //Tie OC4 to RP15*/	