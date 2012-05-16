#define FCY     40000000UL 

#include <stdio.h>
#include <p33Fj128GP202.h>
#include <libpic30.h> 

#define PAGESIZE 0x00ff

void Setup_I2C(void)
{	
	TRISBbits.TRISB9 = 1;
	TRISBbits.TRISB8 = 1;
	//This function will initialize the I2C(1) peripheral.
	
	//Set the I2C(1) BRG Baud Rate.
	//(((1/400KHz)-130ns)x40MIPs)-2 = 93
	I2C1BRG = 93; 
	

	//Now we will initialise the I2C peripheral for Master Mode, No Slew Rate
	//Control, SMbus levels, and leave the peripheral switched off.
	
	I2C1CONbits.I2CEN = 0;
	I2C1CONbits.I2CSIDL = 0;
	I2C1CONbits.SCLREL = 1;
	I2C1CONbits.IPMIEN = 0;
	I2C1CONbits.A10M = 0;
	I2C1CONbits.DISSLW = 1;
	I2C1CONbits.SMEN = 0;
	I2C1CONbits.GCEN = 0;
	I2C1CONbits.STREN = 0;
	I2C1CONbits.ACKDT = 0;
	I2C1CONbits.ACKEN = 0;
	I2C1CONbits.RCEN = 0;
	I2C1CONbits.PEN = 0;
	I2C1CONbits.RSEN = 0;
	I2C1CONbits.SEN = 0;
	
	//Clearing the recieve and transmit buffers
	I2C1RCV = 0x0000;
	I2C1TRN = 0x0000;
	
	//Now we can enable the peripheral	
	I2C1CONbits.I2CEN = 1;
	
	printf("\nI2C Setup Complete");
}

unsigned int StartI2C(void)
{
	//This function generates an I2C start condition and returns status 
	//of the Start.

	I2C1CONbits.SEN = 1;		//Generate Start Condition
	Nop();
	while (I2C1CONbits.SEN);	//Wait for Start Condition
	//return(I2C1STATbits.S);	//Optionally return status
}


/*********************************************************************
* Function:        RestartI2C()
* Overview:		Generates a restart condition and optionally returns status
********************************************************************/
unsigned int RestartI2C(void)
{
	//This function generates an I2C Restart condition and returns status 
	//of the Restart.

	I2C1CONbits.RSEN = 1;		//Generate Restart		
	Nop();
	while (I2C1CONbits.RSEN);	//Wait for restart	
	//return(I2C1STATbits.S);	//Optional - return status
}


/*********************************************************************
* Function:        StopI2C()
* Overview:		Generates a bus stop condition
********************************************************************/
unsigned int StopI2C(void)
{
	//This function generates an I2C stop condition and returns status 
	//of the Stop.

	I2C1CONbits.PEN = 1;		//Generate Stop Condition
	Nop();
	while (I2C1CONbits.PEN);	//Wait for Stop
	//return(I2C1STATbits.P);	//Optional - return status
}


/*********************************************************************
* Function:        WriteI2C()
* Overview:		Writes a byte out to the bus
********************************************************************/
unsigned int WriteI2C(unsigned char byte)
{
	//This function transmits the byte passed to the function
	//while (I2C1STATbits.TRSTAT);	//Wait for bus to be idle
	I2C1TRN = byte;					//Load byte to I2C1 Transmit buffer
	Nop();
	while (I2C1STATbits.TBF);		//wait for data transmission

}


/*********************************************************************
* Function:        IdleI2C()
* Overview:		Waits for bus to become Idle
********************************************************************/
unsigned int IdleI2C(void)
{
	while (I2C1STATbits.TRSTAT);		//Wait for bus Idle
}

/*********************************************************************
* Function:        ACKStatus()
* Output:		Acknowledge Status.
* Overview:		Return the Acknowledge status on the bus
********************************************************************/
unsigned int ACKStatus(void)
{
	return (!I2C1STATbits.ACKSTAT);		//Return Ack Status
}


/*********************************************************************
* Function:        NotAckI2C()
* Overview:		Generates a NO Acknowledge on the Bus
********************************************************************/
unsigned int NotAckI2C(void)
{
	I2C1CONbits.ACKDT = 1;			//Set for NotACk
	I2C1CONbits.ACKEN = 1;
	while(I2C1CONbits.ACKEN);		//wait for ACK to complete
	I2C1CONbits.ACKDT = 0;			//Set for NotACk
}


/*********************************************************************
* Function:        AckI2C()
* Overview:		Generates an Acknowledge.
********************************************************************/
unsigned int AckI2C(void)
{
	I2C1CONbits.ACKDT = 0;			//Set for ACk
	I2C1CONbits.ACKEN = 1;
	while(I2C1CONbits.ACKEN);		//wait for ACK to complete
}

/*********************************************************************
* Function:        getI2C()
* Input:		None.
* Output:		contents of I2C1 receive buffer.
* Overview:		Read a single byte from Bus
********************************************************************/
unsigned int getI2C(void)
{
	I2C1CONbits.RCEN = 1;			//Enable Master receive
	Nop();
	while(!I2C1STATbits.RBF);		//Wait for receive buffer to be full
	return(I2C1RCV);				//Return data in buffer
}

/*********************************************************************
* Function:       getsI2C()
* Input:		array pointer, Length.
* Overview:		read Length number of Bytes into array
********************************************************************/
unsigned int getsI2C(unsigned char *rdptr, unsigned char Length)
{
	while (Length --)
	{
		*rdptr++ = getI2C();		//get a single byte
		
		if(I2C1STATbits.BCL)		//Test for Bus collision
		{
			return(-1);
		}

		if(Length)
		{
			AckI2C();				//Acknowledge until all read
		}
	}
	return(0);
}


/*********************************************************************
* Function:        EEAckPolling()
* Input:		Control byte.
* Output:		Error state.
* Overview:		Polls the bus for an Acknowledge from device
********************************************************************/
unsigned int EEAckPolling(unsigned char control)
{
	IdleI2C();				//wait for bus Idle
	StartI2C();				//Generate Start condition
	
	if(I2C1STATbits.BCL)
	{
		return(-1);			//Bus collision, return
	}

	else
	{
		if(WriteI2C(control))
		{
			return(-3);		//error return
		}

		IdleI2C();			//wait for bus idle
		if(I2C1STATbits.BCL)
		{
			return(-1);		//error return
		}

		while(ACKStatus())
		{
			RestartI2C();	//generate restart
			if(I2C1STATbits.BCL)
			{
				return(-1);	//error return
			}

			if(WriteI2C(control))
			{
				return(-3);
			}

			IdleI2C();
		}
	}
	StopI2C();				//send stop condition
	if(I2C1STATbits.BCL)
	{
		return(-1);
	}
	return(0);
}


/*********************************************************************
* Function:        putstringI2C()
* Input:		pointer to array.
* Overview:		writes a string of data upto PAGESIZE from array
********************************************************************/
unsigned int putstringI2C(unsigned char *wrptr)
{
	unsigned char x;

	for(x = 0; x < PAGESIZE; x++)		//Transmit Data Until Pagesize
	{	
		if(WriteI2C(*wrptr))			//Write 1 byte
		{
			return(-3);				//Return with Write Collision
		}
		IdleI2C();					//Wait for Idle bus
		if(I2C1STATbits.ACKSTAT)
		{
			return(-2);				//Bus responded with Not ACK
		}
		wrptr++;
	}
	return(0);
}


/*********************************************************************
* Function:        LDByteWriteI2C()
* Input:		Control Byte, 8 - bit address, data.
* Overview:		Write a byte to low density device at address LowAdd
********************************************************************/
unsigned int LDByteWriteI2C(unsigned char ControlByte, unsigned char LowAdd, unsigned char data)
{
	unsigned int ErrorCode1;
	unsigned int ErrorCode2;

	IdleI2C();						//Ensure Module is Idle	
	StartI2C();						//Generate Start COndition
	WriteI2C(ControlByte);			//Write Control byte
	IdleI2C();
	
	ErrorCode1 = ACKStatus();		//Return ACK Status
	
	WriteI2C(LowAdd);				//Write Low Address
	IdleI2C();

	ErrorCode2 = ACKStatus();		//Return ACK Status

	WriteI2C(data);					//Write Data
	IdleI2C();
	
	StopI2C();						//Initiate Stop Condition
	//EEAckPolling(ControlByte);		//Perform ACK polling
	if(ErrorCode1 == 0) { printf("ACK 1 not recieved"); }
	if(ErrorCode2 == 0) { printf("ACK 2 not recieved"); }
	//return(ErrorCode);
}


/*********************************************************************
* Function:        LDByteReadI2C()
* Input:		Control Byte, Address, *Data, Length.
* Overview:		Performs a low density read of Length bytes and stores in *Data array
*				starting at Address.
********************************************************************/
unsigned int LDByteReadI2C(unsigned char ControlByte, unsigned char Address, unsigned char *Data, unsigned char Length)
{
	IdleI2C();					//wait for bus Idle
	StartI2C();					//Generate Start Condition
	WriteI2C(ControlByte);		//Write Control Byte
	IdleI2C();					//wait for bus Idle
	WriteI2C(Address);			//Write start address
	IdleI2C();					//wait for bus Idle

	RestartI2C();				//Generate restart condition
	WriteI2C(ControlByte | 0x01);	//Write control byte for read
	IdleI2C();					//wait for bus Idle

	getsI2C(Data, Length);		//read Length number of bytes
	NotAckI2C();				//Send Not Ack
	StopI2C();					//Generate Stop
}

/*********************************************************************
* Function:        LDPageWriteI2C()
* Input:			ControlByte, LowAdd, *wrptr.
* Overview:		Write a page of data from array pointed to be wrptr
*				starting at LowAdd
* Note:			LowAdd must start on a page boundary
********************************************************************/
unsigned int LDPageWriteI2C(unsigned char ControlByte, unsigned char LowAdd, unsigned char *wrptr)
{
	IdleI2C();					//wait for bus Idle
	StartI2C();					//Generate Start condition
	WriteI2C(ControlByte);		//send controlbyte for a write
	IdleI2C();					//wait for bus Idle
	WriteI2C(LowAdd);			//send low address
	IdleI2C();					//wait for bus Idle
	putstringI2C(wrptr);		//send data
	IdleI2C();					//wait for bus Idle
	StopI2C();					//Generate Stop
	return(0);
}



/*********************************************************************
* Function:        LDSequentialReadI2C()
* Input:			ControlByte, address, *rdptr, length.
* Overview:		Performs a sequential read of length bytes starting at address
*				and places data in array pointed to by *rdptr
********************************************************************/
unsigned int LDSequentialReadI2C(unsigned char ControlByte, unsigned char address, unsigned char *rdptr, unsigned char length)
{
	IdleI2C();						//Ensure Module is Idle
	StartI2C();						//Initiate start condition
	WriteI2C(ControlByte);			//write 1 byte
	IdleI2C();						//Ensure module is Idle
	WriteI2C(address);				//Write word address
	IdleI2C();						//Ensure module is idle
	RestartI2C();					//Generate I2C Restart Condition
	WriteI2C(ControlByte | 0x01);	//Write 1 byte - R/W bit should be 1 for read
	IdleI2C();						//Ensure bus is idle
	getsI2C(rdptr, length);			//Read in multiple bytes
	NotAckI2C();					//Send Not Ack
	StopI2C();						//Send stop condition
	return(0);
}

