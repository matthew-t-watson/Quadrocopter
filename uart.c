#include <stdio.h>
#include <p33Fj128GP202.h>

#define  PPSUnlock                   __builtin_write_OSCCONL(OSCCON & 0xbf) 
#define  PPSLock                     __builtin_write_OSCCONL(OSCCON | 0x40)

char lookup[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
int val1, val2, val3, val4;

void Setup_UART1()
{
	
	PPSUnlock;
	RPINR18bits.U1RXR = 0b0101; //Tie RX to RP5
	TRISBbits.TRISB5 = 1;	//Set RB5 as input
	RPOR3bits.RP6R = 0b00011; //Tie TX to RP6
	TRISBbits.TRISB6 = 0;	//Set RB6 as output
	PPSLock;

	U1MODEbits.UARTEN = 0; // Bit15 TX, RX DISABLED, ENABLE at end of func
	//U1MODEbits.notimplemented; // Bit14
	U1MODEbits.USIDL = 0; // Bit13 Continue in Idle
	U1MODEbits.IREN = 0; // Bit12 No IR translation
	U1MODEbits.RTSMD = 0; // Bit11 Simplex Mode
	//U1MODEbits.notimplemented; // Bit10
	U1MODEbits.UEN = 0; // Bits8,9 TX,RX enabled, CTS,RTS not
	U1MODEbits.WAKE = 0; // Bit7 No Wake up (since we don't sleep here)
	U1MODEbits.LPBACK = 0; // Bit6 No Loop Back
	U1MODEbits.ABAUD = 0; // Bit5 No Autobaud (would require sending '55')
	U1MODEbits.URXINV = 0; // Bit4 IdleState = 1 (for dsPIC)
	U1MODEbits.BRGH = 0; // Bit3 16 clocks per bit period
	U1MODEbits.PDSEL = 0b00; // Bits1,2 8bit, no Parity
	U1MODEbits.STSEL = 0; // Bit0 One Stop Bit

	// Load a value into Baud Rate Generator.
	// See section 17.3.1 of datasheet.
	// U2BRG = (Fcy/(16*BaudRate))-1
	// U2BRG = ((79.2275E6/2)/(16*57600))-1
	// U2BRG = 41.98367
	U1BRG = 43; // 43 80Mhz osc, 57600 Baud

	// Load all values in for U1STA SFR
	U1STAbits.UTXISEL1 = 0; //Bit15 Int when Char is transferred (1/2 config!)
	U1STAbits.UTXINV = 0; //Bit14 N/A, IRDA config
	U1STAbits.UTXISEL0 = 0; //Bit13 Other half of Bit15
	//U2STAbits.notimplemented = 0; //Bit12
	U1STAbits.UTXBRK = 0; //Bit11 Disabled
	U1STAbits.UTXEN = 0; //Bit10 TX pins controlled by periph
	U1STAbits.UTXBF = 0; //Bit9 *Read Only Bit*	
	U1STAbits.TRMT = 0; //Bit8 *Read Only bit*
	U1STAbits.URXISEL = 0; //Bits6,7 Int. on character recieved
	U1STAbits.ADDEN = 0; //Bit5 Address Detect Disabled
	U1STAbits.RIDLE = 0; //Bit4 *Read Only Bit*
	U1STAbits.PERR = 0; //Bit3 *Read Only Bit*
	U1STAbits.FERR = 0; //Bit2 *Read Only Bit*
	U1STAbits.OERR = 0; //Bit1 *Read Only Bit*
	U1STAbits.URXDA = 0; //Bit0 *Read Only Bit*
	
	IPC7 = 0x4400; //Mid Range Interrupt Priority level, no urgent reason
	IFS0bits.U1TXIF = 0; //Clear the Transmit Interrupt Flag
	IEC0bits.U1TXIE = 0; //Disable Transmit Interrupts
	IFS0bits.U1RXIF = 0; //Clear the Recieve Interrupt Flag
	IEC0bits.U1RXIE = 0; //Disable Recieve Interrupts

	U1MODEbits.UARTEN = 1; //Turn the peripheral on
	U1STAbits.UTXEN = 1;
	
	printf("\nUART Setup Complete");
}

void transmit_char(char input) //Transmit single char
{
	while(U1STAbits.UTXBF == 1){}
	U1TXREG = input;	
}

void recieve_char()
{
	while(U1STAbits.URXDA==0){} //Wait for new data
	return(U1RXREG);	
}	
	