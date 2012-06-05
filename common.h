	extern signed int ACCEL_XOUT;
	extern signed int ACCEL_YOUT;
	extern signed int ACCEL_ZOUT;
	extern float GYRO_XRATE;
	extern float GYRO_YRATE;
	extern float GYRO_ZRATE;
	extern int GYRO_XRATERAW;
	extern int GYRO_YRATERAW;
	extern int GYRO_ZRATERAW;
	
	extern unsigned char GYRO_XOUT_L;
	extern unsigned char GYRO_XOUT_H;
	extern unsigned char GYRO_YOUT_L;
	extern unsigned char GYRO_YOUT_H;
	extern unsigned char GYRO_ZOUT_L;
	extern unsigned char GYRO_ZOUT_H;
	extern signed int GYRO_XOUT;
	extern signed int GYRO_YOUT;
	extern signed int GYRO_ZOUT;	
	
	extern unsigned char ACCEL_XOUT_L;
	extern unsigned char ACCEL_XOUT_H;
	extern unsigned char ACCEL_YOUT_L;
	extern unsigned char ACCEL_YOUT_H;
	extern unsigned char ACCEL_ZOUT_L;
	extern unsigned char ACCEL_ZOUT_H;
	
	extern signed long GYRO_XOUT_OFFSET_1000SUM;
	extern signed long GYRO_YOUT_OFFSET_1000SUM;
	extern signed long GYRO_ZOUT_OFFSET_1000SUM;
	
	extern float GYRO_XANGLE;
	extern float GYRO_YANGLE;
	extern float GYRO_ZANGLE;
	extern long GYRO_XANGLERAW;
	extern long GYRO_YANGLERAW;
	extern long GYRO_ZANGLERAW;
	extern float ACCEL_XANGLE;
	extern float ACCEL_YANGLE;
	extern float ACCEL_ZANGLE;
	
	extern float KALMAN_XANGLE;
	extern float KALMAN_YANGLE;
	extern float KALMAN_ZANGLE;
	
	extern signed int GYRO_XOUT_OFFSET;
	extern signed int GYRO_YOUT_OFFSET;
	extern signed int GYRO_ZOUT_OFFSET;
	
	extern float dt;
	
	extern float PID_XOUTPUT;
	extern float PID_YOUTPUT;
	extern float PID_ZOUTPUT;
	
	extern float TARGET_XANGLE;
	extern float TARGET_YANGLE;
	extern float TARGET_ZRATE;
	
	extern float COMPLEMENTARY_XANGLE;
	extern float COMPLEMENTARY_XANGLEPREV;
	extern float COMPLEMENTARY_YANGLE;
	extern float COMPLEMENTARY_YANGLEPREV;
	
	extern float throttle;
	
	extern float KP;
	extern float KD;
	extern float KI;
	extern float ZKP;
	extern float ZKD;
	
	extern float XERROR;
	extern float YERROR;
	extern float ZERROR;
	
	extern float OC1_output;
	extern float OC2_output;
	extern float OC3_output;
	extern float OC4_output;
	
	extern float XINTEGRAL;
	extern float YINTEGRAL;

	extern int throttle_input;
	extern int yaw_input;
	extern int pitch_input;
	extern int roll_input;