#include <stdio.h>
#include <p33Fj128GP202.h>
#include "C:\Users\Matt\Quadrocopter\common.h"
#define FCY     40000000UL //Required for built in delay function
#include <libpic30.h> 


#define MOTOR_MIN 5350.0
#define MOTOR_MAX 9500.0
#define MOTOR_RANGE 4150.0 

float PREVIOUS_XERROR = 0;
float PREVIOUS_YERROR = 0;
float PREVIOUS_ZERROR = 0;
float XDIFFERENTIAL = 0;
float YDIFFERENTIAL = 0;
float ZDIFFERENTIAL = 0;




/*
positive x axis is forward
Motor 1 is left of x
Motor 4 is right of x
Motor 2 is next to 1
Motor 3 is next to 4
*/

/*void update_motors_PWM()
{
	if(throttle==0.0)
	{
		OC1RS = 700;
		OC2RS = 700;
		OC3RS = 700;
		OC4RS = 700;	
	}
	else
	{
		OC1RS = 0.5*PID_XOUTPUT + -0.5*PID_YOUTPUT + MOTOR_MIN + MOTOR_RANGE*throttle;
		OC4RS = -0.5*PID_XOUTPUT + -0.5*PID_YOUTPUT + MOTOR_MIN + MOTOR_RANGE*throttle;
		OC2RS = 0.5*PID_XOUTPUT + 0.5*PID_YOUTPUT + MOTOR_MIN + MOTOR_RANGE*throttle;
		OC3RS = -0.5*PID_XOUTPUT + 0.5*PID_YOUTPUT + MOTOR_MIN + MOTOR_RANGE*throttle;
		
		if(OC1RS > 1.5*throttle*MOTOR_RANGE + MOTOR_MIN)
		{OC1RS = 1.5*throttle*MOTOR_RANGE + MOTOR_MIN;}
		if(OC2RS > 1.5*throttle*MOTOR_RANGE + MOTOR_MIN)
		{OC2RS = 1.5*throttle*MOTOR_RANGE + MOTOR_MIN;}
		if(OC3RS > 1.5*throttle*MOTOR_RANGE + MOTOR_MIN)
		{OC3RS = 1.5*throttle*MOTOR_RANGE + MOTOR_MIN;}
		if(OC4RS > 1.5*throttle*MOTOR_RANGE + MOTOR_MIN)
		{OC4RS = 1.5*throttle*MOTOR_RANGE + MOTOR_MIN;}
		
				
		if(OC1RS > MOTOR_MAX) {OC1RS = MOTOR_MAX;}
		if(OC2RS > MOTOR_MAX) {OC2RS = MOTOR_MAX;}
		if(OC3RS > MOTOR_MAX) {OC3RS = MOTOR_MAX;}
		if(OC4RS > MOTOR_MAX) {OC4RS = MOTOR_MAX;}
		
		if(OC1RS < MOTOR_MIN) {OC1RS = MOTOR_MIN;}
		if(OC2RS < MOTOR_MIN) {OC2RS = MOTOR_MIN;}
		if(OC3RS < MOTOR_MIN) {OC3RS = MOTOR_MIN;}
		if(OC4RS < MOTOR_MIN) {OC4RS = MOTOR_MIN;}
	}
}*/

void Calibrate_ESC_Endpoints()
{
	int x = 0;
	for (x=0; x<400; x++)
	{
		OC1R = MOTOR_MAX;
		OC2R = MOTOR_MAX;
		OC3R = MOTOR_MAX;
		OC4R = MOTOR_MAX;
		output_compare_fire();
		__delay_ms(2.5);
		LATAbits.LATA0 = !LATAbits.LATA0;
		
	}
	for (x=0; x<400; x++)
	{
		OC1R = MOTOR_MIN;
		OC2R = MOTOR_MIN;
		OC3R = MOTOR_MIN;
		OC4R = MOTOR_MIN;
		output_compare_fire();
		__delay_ms(2.5);
		LATAbits.LATA0 = !LATAbits.LATA0;
	}
	printf("\nESC endpoints calibrated");
}	

void update_motors_single_shot()
{
	if(throttle==0.0)
	{
		OC1R = 700;
		OC2R = 700;
		OC3R = 700;
		OC4R = 700;
		output_compare_fire();
	}
	else
	{
		OC1_output = 0.7071*PID_XOUTPUT + -0.7071*PID_YOUTPUT + PID_ZOUTPUT + MOTOR_MIN + MOTOR_RANGE*throttle;
		OC2_output = 0.7071*PID_XOUTPUT + 0.7071*PID_YOUTPUT - PID_ZOUTPUT + MOTOR_MIN + MOTOR_RANGE*throttle;
		OC3_output = -0.7071*PID_XOUTPUT + 0.7071*PID_YOUTPUT + PID_ZOUTPUT + MOTOR_MIN + MOTOR_RANGE*throttle;
		OC4_output = -0.7071*PID_XOUTPUT + -0.7071*PID_YOUTPUT - PID_ZOUTPUT + MOTOR_MIN + MOTOR_RANGE*throttle;
						
		if(OC1_output > MOTOR_MAX) {OC1_output = MOTOR_MAX;}
		if(OC2_output > MOTOR_MAX) {OC2_output = MOTOR_MAX;}
		if(OC3_output > MOTOR_MAX) {OC3_output = MOTOR_MAX;}
		if(OC4_output > MOTOR_MAX) {OC4_output = MOTOR_MAX;}
		
		if(OC1_output < MOTOR_MIN) {OC1_output = MOTOR_MIN;}
		if(OC2_output < MOTOR_MIN) {OC2_output = MOTOR_MIN;}
		if(OC3_output < MOTOR_MIN) {OC3_output = MOTOR_MIN;}
		if(OC4_output < MOTOR_MIN) {OC4_output = MOTOR_MIN;}
		
		OC1R = OC1_output;
		OC2R = OC2_output;
		OC3R = OC3_output;
		OC4R = OC4_output;
		output_compare_fire();		
	}
}

void update_PID()
{
	PREVIOUS_XERROR = XERROR;
	PREVIOUS_YERROR = YERROR;
	PREVIOUS_ZERROR = ZERROR;
	
	XERROR = TARGET_XANGLE - COMPLEMENTARY_XANGLE;
	YERROR = TARGET_YANGLE - COMPLEMENTARY_YANGLE;
	ZERROR = TARGET_ZRATE - GYRO_ZRATE;

	XDIFFERENTIAL = (XERROR - PREVIOUS_XERROR)/dt;	
	YDIFFERENTIAL = (YERROR - PREVIOUS_YERROR)/dt;
	ZDIFFERENTIAL = (ZERROR - PREVIOUS_ZERROR)/dt;
	
	XINTEGRAL += XERROR*dt;
	YINTEGRAL += YERROR*dt;
	/*if(XINTEGRAL > 0.5) {XINTEGRAL = 0.5;}
	else if(XINTEGRAL < -0.5) {XINTEGRAL = -0.5;}
	if(YINTEGRAL > 0.5) {YINTEGRAL = 1;}
	else if(YINTEGRAL < -0.5) {YINTEGRAL = -0.5;}*/
	
	if(XINTEGRAL > 50) {XINTEGRAL = 50;}
	else if(XINTEGRAL < -50) {XINTEGRAL = -50;}
	if(YINTEGRAL > 50) {YINTEGRAL = 50;}
	else if(YINTEGRAL < -50) {YINTEGRAL = -50;}
	
	PID_XOUTPUT = XERROR*KP + XDIFFERENTIAL*KD + XINTEGRAL*KI;
	PID_YOUTPUT = YERROR*KP + YDIFFERENTIAL*KD + YINTEGRAL*KI;
	PID_ZOUTPUT = ZERROR*ZKP; + ZDIFFERENTIAL*ZKD;
	if(PID_ZOUTPUT > 1000){PID_ZOUTPUT = 1000;}
	else if (PID_ZOUTPUT < -1000){PID_ZOUTPUT = -1000;}	
}	