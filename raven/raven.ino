#include "raven.h"

#include "motor.h"
Motor m_FR(ESC_PIN_FR);
Motor m_FL(ESC_PIN_FL);
Motor m_BR(ESC_PIN_BR);
Motor m_BL(ESC_PIN_BL);

/*
 * Use br3ttb PID library
 * https://github.com/br3ttb/Arduino-PID-Library
 */
#include "PID_v1.h"
PID *pid_FR;
PID *pid_FL;
PID *pid_BR;
PID *pid_BL;

/*
 * Use jrowberg I2Cdev library for MPU6050, HMC5883L & BMP085/180
 * https://github.com/jrowberg/i2cdevlib
 */ 
#include "I2Cdev.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "MPU6050.h"
//#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
// Calibration offsets: computed w/ I2Cdev IMU_Zero sketch
int off_ax = -989, off_ay = -492, off_az = 1557;
int off_gx = 36, off_gy = 1, off_gz = 26;

#include "HMC5883L.h"
HMC5883L compass;

// Calibration offsets
// Paris
float off_decl_deg = 0, off_decl_min = 57.96; // declinaison angle degrees/minutes for Paris
float off_decl = 0.0; // declinaison angle

#include "BMP085.h"
BMP085 barometer;

// Drone attitude
typedef struct {
	float x_acc, y_acc, z_acc;	// acceleration (m.s-2)
	float x_rate, y_rate, z_rate;	// angular speeds (rads.s-1)
	
	float pitch, roll;		// pitch/roll angles (rads)

	float heading, heading_p;	// current & previous heading (rads)
	float rspeed;			// Angular speed (rads.s-1)
	
	float altitude, altitude_p;	// current & previous altitude (m)
	float vspeed;			// vertical speed (m.s-1)

	float pressure;			// pressure (pa)
	float temperature;		// temperature (deg. C)
} attitude_t;

attitude_t attitude;

const float RAD2DEG = 180.0/M_PI;
const float DEG2RAD = M_PI/180.0;

/* 
 * Use bolderflight SBUS library for Teensy 
 * https://github.com/bolderflight/SBUS.git
 */
#include "SBUS.h"
// FRSky XSR receiver in SBUS mode on Serial3
SBUS xsr(Serial3);
// SBUS channels & data
float channels[16];
uint8_t failSafe;
uint16_t lostFrames = 0;

// Loop timer
unsigned long timer = 0;
unsigned long dt_loop = 0;

/* ----- Get drone attitude ----- */
void get_attitude(unsigned long ms)
{
	// delta time (in seconds)
	float dt = ms / 1000.0;
	
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	// Get raw values
	mpu.getAcceleration(&ax, &ay, &az);
	mpu.getRotation(&gx, &gy, &gz);

	// Accelerations (m.s-2)
	attitude.x_acc = (9.81 * ax) / 16384.0;
	attitude.y_acc = (9.81 * ay) / 16384.0;
	attitude.z_acc = (9.81 * az) / 16384.0;
	
	// Angular speeds (rads.s-1)
	// The sensor returns deg.s-1
        attitude.x_rate = (gx * DEG2RAD) / 131.0; 
        attitude.y_rate = (gy * DEG2RAD)/ 131.0;
        attitude.z_rate = (gz * DEG2RAD)/ 131.0;

        // Orientation of the accelerometer relative to the earth
        float x_angle = atan2(ay, az);
        float y_angle = atan2(-ax, az);

        // Pitch and roll angles (rads) corrected w/ complementary filter
        attitude.roll = 0.96 * (attitude.roll + attitude.x_rate * dt) + 0.04 * x_angle; 
        attitude.pitch = 0.96 * (attitude.pitch + attitude.y_rate * dt) + 0.04 * y_angle; 

	// Backup previous heading
	attitude.heading_p = attitude.heading;

	int16_t mx, my, mz, temp;

	while(!compass.getReadyStatus());
	compass.getHeading(&mx, &my, &mz);

	if (mx == -4096 || mx == -4096 || mz == -4096) {
		Serial.println("Compass overflow");
	} else {
	
		// Swap X/Y to match MPU disposal on the board
		temp = mx;
		mx = -my;
		my = temp;
		
		float cosRoll = cos(attitude.roll);
	  	float sinRoll = sin(attitude.roll);  
	  	float cosPitch = cos(attitude.pitch);
	  	float sinPitch = sin(attitude.pitch);
	  
	  	// Tilt compensation
	  	float Xh = mx * cosPitch + mz * sinPitch;
	  	float Yh = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;
	 
	  	attitude.heading = atan2(Yh, Xh) + off_decl;
	
		// Correct angle if necessary
		if (attitude.heading < 0) 
			attitude.heading += 2 * PI; 
	  	if (attitude.heading > 2 * PI) 
	  		attitude.heading -= 2 * PI;  
	
	  	// Angular speed
	  	attitude.rspeed = (attitude.heading - attitude.heading_p) / dt;
	}
	
  	// request temperature
    	barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    	// wait appropriate time for conversion (4.5ms delay)
    	unsigned long lastMicros = micros();
    	while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    	// read calibrated temperature value in degrees Celsius
    	attitude.temperature = barometer.getTemperatureC();

	// request pressure (3x oversampling mode, high detail, 23.5ms delay)
    	barometer.setControl(BMP085_MODE_PRESSURE_3);
    	while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    	// read calibrated pressure value in Pascals (Pa)
    	attitude.pressure = barometer.getPressure();

    	// calculate absolute altitude in meters based on known pressure
    	attitude.altitude_p = attitude.altitude;  
    	attitude.altitude = barometer.getAltitude(attitude.pressure);

	// Vertical speed
    	attitude.vspeed = (attitude.altitude - attitude.altitude_p) / dt;
}	

/* ----- Calibrate ----- */
#define N_CALIBRATION 2000

void calibrate()
{
	delay(500);

	int16_t ax, ay, az, gx, gy, gz;
	
	float sum_ax = 0, sum_ay = 0, sum_az = 0;
	float sum_gx = 0, sum_gy = 0, sum_gz = 0;
	
	for(int i = 0; i < N_CALIBRATION; i ++) {
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		sum_ax += ax;
		sum_ay += ay;
		sum_az += az;

		sum_gx += gx;
		sum_gy += gy;
		sum_gz += gz;

		delay(4);
	}

	off_ax = sum_ax / N_CALIBRATION;
	off_ay = sum_ay / N_CALIBRATION;
	off_az = sum_az / N_CALIBRATION;

	off_gx = sum_gx / N_CALIBRATION;
	off_gy = sum_gy / N_CALIBRATION;
	off_gz = sum_gz / N_CALIBRATION;	
}

/* ----- Flight controller ----- */

/*
 * Full manual control of the drone, based only on commands received from user.
 * Don't apply any compensation, free flight style !
 *
 * To compute each motor speed, use the following rules:
 * 
 *  pitch < 0 -> nose raises up -> front motors speed increases & back motors speed decreases
 *  pitch > 0 -> nose dips -> front motors speed decreases & back motors speed increases
 *   roll < 0 -> rolls left -> left motors speed decreases & right motors speed increases
 *   roll > 0 -> rolls right -> left motors speed increases & right motors speed decreases
 *    yaw < 0 -> rotate left -> front-right/back-left motors speed increases & front-left/back-right motors speed decreases
 *    yaw > 0 -> rotate right -> front-right/back-left motors speed decreases & front-left/back-right motors speed increases
 */
void set_motor_speed_manual(float cmd_throttle, float cmd_yaw, float cmd_pitch, float cmd_roll)
{
	// Keep some room for motors adjustement
	if (cmd_throttle > 0.9)
		cmd_throttle = 0.9;

	int16_t front_right_speed, front_left_speed, back_right_speed, back_left_speed;
	front_right_speed = front_left_speed = back_right_speed = back_left_speed = ESC_PULSE_MIN_WIDTH + cmd_throttle * (ESC_PULSE_MAX_WIDTH - ESC_PULSE_MIN_WIDTH);

/*	
	front_right_speed  += 0.15 * (- roll - pitch - yaw) / 3;
	back_left_speed    += 0.15 * (  roll + pitch - yaw) / 3;
	front_left_speed   += 0.15 * (  roll - pitch + yaw) / 3;
	back_right_speed   += 0.15 * (- roll + pitch + yaw) / 3;
*/
	
	m_FR.set_speed(front_right_speed);
	m_FL.set_speed(front_left_speed);
	m_BR.set_speed(back_right_speed);
	m_BL.set_speed(back_left_speed);
}

/* -----  Setup ----- */
void setup()
{
	// Start serial console
	Serial.begin(9600);

	Serial.print(F("raven v"));
	Serial.print(RAVEN_VERSION);
	Serial.println(F(" starting"));

	Serial.println(F("> Initializing SBUS RX"));
	xsr.begin();
	
	// Join I2C bus (I2Cdev library doesn't do this automatically)
	Serial.println(F("> Initializing I2C bus"));
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock

	delay(2000);

	// Clear attitude data
	memset(&attitude, 0, sizeof(attitude_t));

	// Initialize compass
	Serial.println("> Initializing compass");
	compass.initialize();
	
	// Check connection w/ compass
	if (!compass.testConnection()) {
		Serial.println(F("  Connection to HMC5883L failed !"));
		while(true);
	}

	compass.setMode(HMC5883L_MODE_CONTINUOUS);
	compass.setSampleAveraging(HMC5883L_AVERAGING_8);
	
  	// Configure
	off_decl = (off_decl_deg + (off_decl_min / 60.0)) * (M_PI / 180.0); 
  	
/*  	compass.setRange(HMC5883L_RANGE_1_3GA);
	compass.setMeasurementMode(HMC5883L_CONTINOUS);
	compass.setDataRate(HMC5883L_DATARATE_30HZ);
  	compass.setSamples(HMC5883L_SAMPLES_8);
 	
 	compass.setOffset(off_mag2, off_mag2); 
*/

	// Initialize barometer
	Serial.println("> Initializing barometer");
	barometer.initialize();
	
	// Check connection w/ compass
	if (!barometer.testConnection()) {
		Serial.println(F("  Connection to BMP180 failed !"));
		while(true);
	}

	// Initialize MPU
	Serial.println(F("> Initializing MPU"));
	mpu.initialize();

	// Check connection w/ MPU
	if (!mpu.testConnection()) {
		Serial.println(F("  Connection to MPU6050 failed !"));
		while(true);
	}

	//0 = +/- 250 degrees/sec | 1 = +/- 500 degrees/sec | 2 = +/- 1000 degrees/sec | 3 =  +/- 2000 degrees/sec
	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); 
	//0 = +/- 2g | 1 = +/- 4g | 2 = +/- 8g | 3 =  +/- 16g
  	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   
/*
	Serial.print("  Calibration ");
	Serial.print("    Current offsets: ");
	Serial.print("Acc(");
	Serial.print(mpu.getXAccelOffset());
	Serial.print(",");
	Serial.print(mpu.getYAccelOffset());
	Serial.print(",");
	Serial.print(mpu.getZAccelOffset());
	Serial.print(") Gyr(");
	Serial.print(mpu.getXGyroOffset());
	Serial.print(",");
	Serial.print(mpu.getYGyroOffset());
	Serial.print(",");
	Serial.print(mpu.getZGyroOffset());
	Serial.println(")");
	
	calibrate();
*/
	mpu.setXAccelOffset(off_ax);
	mpu.setYAccelOffset(off_ay);
	mpu.setZAccelOffset(off_az);

	mpu.setXGyroOffset(off_gx);
	mpu.setYGyroOffset(off_gy);
	mpu.setZGyroOffset(off_gz);
/*	
	Serial.print("    New offsets: ");
	Serial.print("Acc(");
	Serial.print(mpu.getXAccelOffset());
	Serial.print(",");
	Serial.print(mpu.getYAccelOffset());
	Serial.print(",");
	Serial.print(mpu.getZAccelOffset());
	Serial.print(") Gyr(");
	Serial.print(mpu.getXGyroOffset());
	Serial.print(",");
	Serial.print(mpu.getYGyroOffset());
	Serial.print(",");
	Serial.print(mpu.getZGyroOffset());
	Serial.println(")");
*/		
	Serial.println(F("Ready"));
	Serial.println(F("ax,ay,az,gx,gy,gz,pitch,roll,heading,rspeed,altitude,vspeed,temperature,pressure"));
	
	// Initialize loop timer
	timer = millis();

	delay(20);
}

// Dump all SBUS channels to serial
void dump_channels()
{
		for(int i = 0; i < 16; i ++) {
			Serial.print("Ch");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(channels[i]);
			Serial.print(" ");
		}

		Serial.println();
}

// Dump attitude to serial
void dump_attitude()
{	
	Serial.print(attitude.x_acc, 1);
	Serial.print(",");
	Serial.print(attitude.y_acc, 1);
	Serial.print(", ");
	Serial.print(attitude.z_acc, 1);
	Serial.print(", ");
	Serial.print(attitude.x_rate * RAD2DEG, 1);
	Serial.print(", ");
	Serial.print(attitude.y_rate * RAD2DEG, 1);
	Serial.print(", ");
	Serial.print(attitude.z_rate * RAD2DEG, 1);
	Serial.print(", ");
	Serial.print(attitude.pitch * RAD2DEG, 1);
	Serial.print(", ");
	Serial.print(attitude.roll * RAD2DEG, 1);
	Serial.print(", ");
	Serial.print(attitude.heading * RAD2DEG, 1);
	Serial.print(", ");
	Serial.print(attitude.rspeed * RAD2DEG, 1);
	Serial.print(", ");
	Serial.print(attitude.altitude);
	Serial.print(", ");
	Serial.print(attitude.vspeed);
	Serial.print(", ");
	Serial.print(attitude.temperature);
	Serial.print(", ");
	Serial.println(attitude.pressure);
}

/* ----- Main loop ----- */
void loop()
{	
	float throttle = 0;
	
	unsigned long start = millis(), dt = start - timer;
	timer = start;
	
	// Get user commands
	if (xsr.readCal(&channels[0], &failSafe, &lostFrames)) {
		
		if (failSafe) {
			// Out of range, stop everything :-/
			// do something ...
//			Serial.println(F("!!! OUT OF RANGE !!!"));				
			m_FR.set_speed(0);
			m_FL.set_speed(0);
			m_BR.set_speed(0);
			m_BL.set_speed(0);
			
//			return;
		}				

		if (channels[CMD_ARMED_ID] >= 0.0) {
			// Translate throttle value between [0;1] rather than [-1;1]
			throttle = MOTOR_ARMED_SPEED + (1.0 - MOTOR_ARMED_SPEED) * (1.0 + channels[CMD_THROTTLE_ID]) * 0.5;
		} 
	}

	get_attitude(dt);
	dump_attitude();

	// Loop execution time
	unsigned long end = millis();
	dt_loop = end - start;
}
