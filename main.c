#include <stdio.h>
#include <p33Fj128GP202.h>
#include "D:\documents\Matthew\mplab\ControlV4\MPU6050.h" //For some strange reason I couldn't get this to function when relative to the root directory, so I just gave in and included the whole path
#include "D:\documents\Matthew\mplab\ControlV4\declarations.h"
#define FCY     40000000UL //Required for built in delay function
#include <libpic30.h>  
#include <math.h>

_FPOR(FPWRT_PWR128 & ALTI2C_OFF) //set power on timer to 128ms
_FOSCSEL(FNOSC_FRCPLL) //set clock for internal OSC with PLL
_FOSC(OSCIOFNC_OFF & POSCMD_NONE & IOL1WAY_OFF ) //no clock output, external OSC disabled, multiple peripheral pin changes
_FWDT(FWDTEN_OFF & WDTPRE_PR32 & WDTPOST_PS64 & WINDIS_OFF) //Watchdog timer off, but can be enabled by software
_FICD(JTAGEN_OFF & ICS_PGD1) //disable JTAG, enable debugging on PGx1 pins


void Setup_Oscillator()
{
	// setup internal clock for 80MHz/40MIPS
	// 7.37/2=3.685*43=158.455/2=79.2275
	CLKDIVbits.PLLPRE=0;        // PLLPRE (N2) 0=/2
	PLLFBD=41;                  // pll multiplier (M) = +2
	CLKDIVbits.PLLPOST=0;       // PLLPOST (N1) 0=/2
	while(!OSCCONbits.LOCK);    // wait for PLL ready	
}

void Zero_Sensors()
{
	float BUFFER_XANGLE = 0;
	float BUFFER_YANGLE = 0;
	int x = 0;
	for(x=0; x<100; x++)
	{
		Get_Accel_Values();
		Get_Accel_Angles();
		BUFFER_XANGLE += ACCEL_XANGLE;
		BUFFER_YANGLE += ACCEL_YANGLE;
		__delay_ms(1);
	}
	COMPLEMENTARY_XANGLE = BUFFER_XANGLE/100.0;
	COMPLEMENTARY_YANGLE = BUFFER_YANGLE/100.0;
	GYRO_XANGLE = BUFFER_XANGLE/100.0;
	GYRO_YANGLE = BUFFER_YANGLE/100.0;
}	

int main(void)
{
	Setup_Oscillator();
	if(RCONbits.WDTO == 1) //If watchdog reset has occured
	{
		Setup_Timer3();
		Setup_OC_Single_Shot();			
		OC1R = 700;
		OC2R = 700;
		OC3R = 700;
		OC4R = 700;
		output_compare_fire();	
			
		Setup_UART1();
		printf("\nSignal Lost");
		while(1)
		{
			LATAbits.LATA0 = !LATAbits.LATA0;
			__delay_ms(100);
		}
	}	
	
	AD1PCFGL = 0xffff; //digital pins
	TRISAbits.TRISA0 = 0;	
	LATAbits.LATA0 = 1; // Set LED high
	
	int error = 1;
	Setup_UART1();
	Setup_I2C();
	do
	{
		Setup_MPU6050();
		MPU6050_Test_I2C();
		error = MPU6050_Check_Registers();
	}
	while(error==1);
	
	Calibrate_Gyros();
	Zero_Sensors();
	
	Setup_Timer1(); //400Hz loop timer
	Setup_Timer2(); //Input capture timer
	Setup_Timer3(); //Output compare timer
	Setup_Timer4(); //dt measuring timer
	Setup_OC_Single_Shot();	
	Setup_IC();	
	RCONbits.SWDTEN = 1; //Enable watchdog timer
	IEC0bits.T1IE = 1; //Enable timer1 interrupt
	
	while(1)
	{
		if(U1STAbits.TRMT == 1)
		{
			//printf("\n%d,	%d,	%d,	%d",throttle_input, yaw_input, pitch_input, roll_input);
			//printf("\n%f	%f	%f", throttle, TARGET_XANGLE, TARGET_YANGLE);
			//printf("\n%u",TMR5);
			//printf("\n%u",OC1R);
			//printf("\n%f,	%f", PID_ZOUTPUT,ZERROR);
			printf("\n%f,%f", COMPLEMENTARY_XANGLE, COMPLEMENTARY_YANGLE);
			//printf("\n%.1f,%.1f,%.1f", GYRO_XANGLE, ACCEL_XANGLE, COMPLEMENTARY_XANGLE);
			//printf("\n%d,%d,%d", ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT);
			//printf("\n%f,%f", ACCEL_XANGLE, ACCEL_YANGLE);
		}
		
		if(U1STAbits.URXDA==1){
			char input = U1RXREG;
			
			if(input == '+')
			{
				throttle = throttle + 0.01;
				printf(",%f", throttle);
			}
			else if (input == '-' && throttle>0)
			{
				throttle = throttle - 0.01;
				printf(",%f", throttle);
			}
			else if (input == '*')
			{				
				IEC0bits.T1IE = 0; //Disable timer1 interrupt
				IEC0bits.IC1IE = 0; //Disable interrupt
				IEC0bits.IC2IE = 0; //Disable interrupt
				IEC1bits.IC7IE = 0; //Disable interrupt
				IEC1bits.IC8IE = 0; //Disable interrupt
				while(1){}
			}
			else if (input == 'z')
			{
				ZKP = ZKP + 1;
				printf(",%f",ZKP);
			}
			else if (input == 'x')
			{
				ZKP = ZKP - 1;
				printf(",%f",ZKP);
			}
			else if (input == 'r')
			{
				__asm__ volatile ("reset"); 
			}
			else if (input == 'c')
			{
				ZKD = ZKD + 1;
				printf(",%f",ZKD);
			}
			else if (input == 'v')
			{
				ZKD = ZKD - 1;
				printf(",%f",ZKD);
			}
			
			else if (input == 'b')
			{
				KI = KI + 1;
				printf(",%f",KI);
			}
			else if (input == 'n')
			{
				KI = KI - 1;
				printf(",%f",KI);
			}
			
			else if (input == 'q')
			{
				TARGET_XANGLE = -20.0;
				printf(",%f",TARGET_XANGLE);
			}
			else if (input == 'w')
			{
				TARGET_XANGLE = 0.0;
				printf(",%f",TARGET_XANGLE);
			}
			else if (input == 'e')
			{
				TARGET_XANGLE = 20.0;
				printf(",%f",TARGET_XANGLE);
			}
		}		
	}
}

