#include <stdio.h>
#include <p33Fj128GP202.h>
#include "D:\documents\Matthew\mplab\ControlV4\common.h"

#define timeConstant 1/2.0
float a = 0.998;

float filter_xterm[3] = {0,0,0};
float filter_yterm[3] = {0,0,0};

//Runs a complementary filter configured via float a
void complementary_filter()
{
	COMPLEMENTARY_XANGLEPREV = COMPLEMENTARY_XANGLE;
	COMPLEMENTARY_XANGLE = (COMPLEMENTARY_XANGLEPREV + GYRO_XRATE*dt)*a + ACCEL_XANGLE*(1-a);
	COMPLEMENTARY_YANGLEPREV = COMPLEMENTARY_YANGLE;
	COMPLEMENTARY_YANGLE = (COMPLEMENTARY_YANGLE + GYRO_YRATE*dt)*a + ACCEL_YANGLE*(1-a);		
}	

//Runs 2nd order complementary filter
void second_order_complementary_filter() //http://code.google.com/p/aeroquad/source/browse/trunk/AeroQuad/Filter.pde?r=143
{
	filter_xterm[0] = (ACCEL_XANGLE - COMPLEMENTARY_XANGLE) * timeConstant * timeConstant;
	filter_yterm[0] = (ACCEL_YANGLE - COMPLEMENTARY_YANGLE) * timeConstant * timeConstant;
  	filter_xterm[2] = (dt * filter_xterm[0]) + filter_xterm[2];
  	filter_yterm[2] = (dt * filter_yterm[0]) + filter_yterm[2];
  	filter_xterm[1] = filter_xterm[2] + (ACCEL_XANGLE - COMPLEMENTARY_XANGLE) * 2 * timeConstant + GYRO_XRATE;
  	filter_yterm[1] = filter_yterm[2] + (ACCEL_YANGLE - COMPLEMENTARY_YANGLE) * 2 * timeConstant + GYRO_YRATE;
  	COMPLEMENTARY_XANGLE = (dt * filter_xterm[1]) + COMPLEMENTARY_XANGLE;
  	COMPLEMENTARY_YANGLE = (dt * filter_yterm[1]) + COMPLEMENTARY_YANGLE;
}	