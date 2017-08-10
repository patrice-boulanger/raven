#include "raven.h"
#include "buzzer.h"
#include "led.h"
#include "motor.h"
#include "PID.h"
/*
 * Use jrowberg I2Cdev library for MPU6050, HMC5883L & BMP085/180
 * https://github.com/jrowberg/i2cdevlib
 */ 
#include "I2Cdev.h"
#include <Wire.h>
#include "MPU6050.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include "BMP085.h"
/* 
 * Use bolderflight SBUS library for Teensy 
 * https://github.com/bolderflight/SBUS.git
 */
#include "SBUS.h"

/* ----- Constants -----  */

// Complementary filter coefficient
#define ALPHA                   0.96

// Number of samples for altitude averaging
#define ALT_SAMPLES             20

// Degrees/radians conversion
const float RAD2DEG = 180.0/M_PI;
const float DEG2RAD = M_PI/180.0;

// Speed interval for throttle
// Divide by 2 to translate throttle from [-1;+1] to [0;1] 
const float speed_scale = 0.5 * (ESC_PULSE_SPEED_FULL_WIDTH - ESC_PULSE_SPEED_0_WIDTH);

/* ----- Variables ----- */

// FRSky XSR receiver in SBUS mode on Serial3
SBUS xsr(Serial3);

// SBUS channels & data
float channels[16];
uint8_t failSafe;
uint16_t lostFrames = 0;

MPU6050 mpu;
// Calibration offsets: computed w/ I2Cdev IMU_Zero sketch
int off_ax = -989, off_ay = -492, off_az = 1557;
int off_gx = 36, off_gy = 1, off_gz = 26;

HMC5883L compass;
// Magnetic declinaison
float off_decl_deg = 0, off_decl_min = 57.96; // Paris
float off_decl = 0.0; // declinaison angle

BMP085 barometer;

/* --- Flight Controller Data --- */
// Drone attitude
typedef struct {
	int16_t ax, ay, az, gx, gy, gz;
	
	float x_acc, y_acc, z_acc;	// acceleration (m.s-2)
	float x_rate, y_rate, z_rate;	// angular speeds (rads.s-1)
	
	float pitch, roll;		// pitch/roll angles (rads)

	int16_t mx, my, mz;
	
	float heading, heading_p;	// current & previous heading (rads)
	float rspeed;			// angular speed (rads.s-1)

	float pressure;			// pressure (pa)
	float temperature;		// temperature (deg. C)
	
	float alt_samples[ALT_SAMPLES]; // Altitude samples
	int alt_idx;			// Current index in buffer
	
	float altitude, altitude_p;	// current & previous altitude (m)
	float vspeed;			// vertical speed (m.s-1)

} attitude_t;

attitude_t attitude;

// Flight controller status
int status;

// Flight mode
int mode;

// Motor arming switch
bool armed = false;
// Buzzer switch
bool buzzer = false;
// Camera declinaison angle
float camera_angle = 0;

// Max. rates for acro mode (rads.s-1)
float yaw_max_rate, pitch_max_rate, roll_max_rate;
// Max. angles for self-level mode (rads)
float pitch_max_angle, roll_max_angle;

PID pid_yaw;
PID pid_roll;
PID pid_pitch;

float yaw_setpoint, pitch_setpoint, roll_setpoint;
int yaw_output, pitch_output, roll_output;

Motor esc_FR(ESC_PIN_FR); // Front-Right CCW
Motor esc_FL(ESC_PIN_FL); // Front-Left  CW
Motor esc_RR(ESC_PIN_RR); // Rear-Right  CW
Motor esc_RL(ESC_PIN_RL); // Rear-Left   CCW

// Base pulse computed from throttle
int16_t base_pulse;
// Pulse width adjustement for each motor, computed from PID
int16_t pulse_fr, pulse_fl, pulse_rr, pulse_rl; 

// Loop timer
unsigned long timer = 0;

// Infinite-Impulse-Response (IIR) single-pole low-pass filter
// https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
float iir_lpf(float previous, float current, float alpha = 0.3)
{
	return previous + alpha * (current - previous);
}

/* ----- Get drone attitude ----- */
void get_attitude(unsigned long ms)
{
	// Delta time in seconds
	float dt = ms / 1000.0;

        // Raw values from MPU
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	
	mpu.getAcceleration(&ax, &ay, &az);
	mpu.getRotation(&gx, &gy, &gz);

	// Accelerations (m.s-2)
	attitude.ax = iir_lpf(attitude.ax, ax);
	attitude.ay = iir_lpf(attitude.ay, ay);
	attitude.az = iir_lpf(attitude.az, az);
	
	attitude.x_acc = (9.81 * attitude.ax) / 16384.0;
	attitude.y_acc = (9.81 * attitude.ay) / 16384.0;
	attitude.z_acc = (9.81 * attitude.az) / 16384.0;
	
	// Angular speeds (rads.s-1)
	// The sensor returns deg.s-1
	attitude.gx = iir_lpf(attitude.gx, gx);
	attitude.gy = iir_lpf(attitude.gy, gy);
	attitude.gz = iir_lpf(attitude.gz, gz);	
	
        attitude.x_rate = (attitude.gx * DEG2RAD) / 131.0; 
        attitude.y_rate = (attitude.gy * DEG2RAD) / 131.0;
        attitude.z_rate = (attitude.gz * DEG2RAD) / 131.0;

        // Orientation of the accelerometer relative to the earth
        float x_angle = atan2(attitude.ay, attitude.az);
        float y_angle = atan2(-attitude.ax, attitude.az);

        // Pitch and roll angles (rads) corrected w/ complementary filter
        attitude.roll = ALPHA * (attitude.roll + attitude.x_rate * dt) + (1 - ALPHA) * x_angle; 
        attitude.pitch = ALPHA * (attitude.pitch + attitude.y_rate * dt) + (1 - ALPHA) * y_angle; 

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

		attitude.mx = iir_lpf(attitude.mx, mx);
		attitude.my = iir_lpf(attitude.my, my);
		attitude.mz = iir_lpf(attitude.mz, mz);
		
		float cosRoll = cos(attitude.roll);
	  	float sinRoll = sin(attitude.roll);  
	  	float cosPitch = cos(attitude.pitch);
	  	float sinPitch = sin(attitude.pitch);
	  
	  	// Tilt compensation (rads)
	  	float Xh = attitude.mx * cosPitch + attitude.mz * sinPitch;
	  	float Yh = attitude.mx * sinRoll * sinPitch + attitude.my * cosRoll - attitude.mz * sinRoll * cosPitch;
	 
	  	attitude.heading = atan2(Yh, Xh) + off_decl;
	
		// Correct angle if necessary
		if (attitude.heading < 0) 
			attitude.heading += 2 * PI;
                      
	  	if (attitude.heading > 2 * PI) 
	  		attitude.heading -= 2 * PI;  
	
	  	// Angular speed (rads.s-1)
	  	attitude.rspeed = (attitude.heading - attitude.heading_p) / dt;
	}
	
  	// Request temperature
    	barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    	// Wait appropriate time for conversion (4.5ms delay)
    	unsigned long lastMicros = micros();
    	while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    	// Read calibrated temperature value (deg. Celsius)
    	attitude.temperature = barometer.getTemperatureC();

	// Request pressure (3x oversampling mode, high detail, 23.5ms delay)
    	barometer.setControl(BMP085_MODE_PRESSURE_3);
    	while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    	// Read calibrated pressure value in Pascals (Pa)
    	attitude.pressure = barometer.getPressure();

    	// Calculate absolute altitude (m) based on known pressure
    	attitude.altitude_p = attitude.altitude;  
	
	attitude.alt_samples[attitude.alt_idx] = barometer.getAltitude(attitude.pressure);
	attitude.alt_idx = (attitude.alt_idx + 1) % ALT_SAMPLES;

	attitude.altitude = 0;
	for(int i = 0; i < ALT_SAMPLES; i ++) 
		attitude.altitude += attitude.alt_samples[i];

	attitude.altitude /= ALT_SAMPLES;
	
	// Vertical speed (m.s-1)
    	attitude.vspeed = (attitude.altitude - attitude.altitude_p) / dt;
}	

/* -----  Setup ----- */
void setup()
{
	// Start serial console
	Serial.begin(9600);

	Serial.print(F("raven v"));
	Serial.print(RAVEN_VERSION);
	Serial.println(F(" starting"));

        // Buzzer
        pinMode(BUZZER_PIN, OUTPUT);
        buzzer_play(BUZZER_START);
        
	// LEDs
	pinMode(LED_GREEN_PIN, OUTPUT);
	pinMode(LED_RED_PIN, OUTPUT);
	pinMode(LED_WHITE_PIN, OUTPUT);

	led_clear();
	led_on(LED_RED_PIN);
	
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
	
  	// Configure
	compass.setGain(HMC5883L_GAIN_1090);
	compass.setMode(HMC5883L_MODE_CONTINUOUS);
	compass.setDataRate(HMC5883L_RATE_30);
	compass.setSampleAveraging(HMC5883L_AVERAGING_8);

	off_decl = (off_decl_deg + (off_decl_min / 60.0)) * DEG2RAD;
  	
	// Initialize barometer
	Serial.println("> Initializing barometer");
	barometer.initialize();
	
	// Check connection w/ barometer
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

        // Configure
	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // +/- 250 degrees/sec 
  	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // +/- 2g

	mpu.setXAccelOffset(off_ax);
	mpu.setYAccelOffset(off_ay);
	mpu.setZAccelOffset(off_az);

	mpu.setXGyroOffset(off_gx);
	mpu.setYGyroOffset(off_gy);
	mpu.setZGyroOffset(off_gz);

        // Configure flight controller

        // Set PID coefficients & min/max values
        pid_yaw.set_kpid(100, 5, 0.0);
        pid_yaw.set_minmax(-200, 200);
        
        pid_pitch.set_kpid(250, 1, 0.0);
        pid_pitch.set_minmax(-200, 200);
        
        pid_roll.set_kpid(250, 1, 0.0);
        pid_roll.set_minmax(-200, 200);

        yaw_max_rate = (30 * DEG2RAD);
        pitch_max_rate = (15 * DEG2RAD);
        roll_max_rate = (15 * DEG2RAD);
        
        pitch_max_angle = (15 * DEG2RAD);
        roll_max_angle = (15 * DEG2RAD);

        mode = FLIGHT_MODE_LEVELED;

	led_sequence("rG__________gR__________");
	
	Serial.print(F("Waiting for TX ... "));
	
        int cnt = 10;
        while(cnt != 0) {
		if (xsr.readCal(&channels[0], &failSafe, &lostFrames) && !failSafe)
                        cnt --;

  		led_update();   
		delay(10);
	}

	Serial.println("OK");

	buzzer_play(BUZZER_READY);
	led_sequence("G__________gR__________rW__________w");

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
/*	Serial.print(attitude.x_acc, 2);
	Serial.print(",");
	Serial.print(attitude.y_acc, 2);
	Serial.print(", ");
	Serial.print(attitude.z_acc, 2);
	Serial.print(", ");
*/	Serial.print(attitude.x_rate * RAD2DEG, 2);
	Serial.print(", ");
	Serial.print(attitude.y_rate * RAD2DEG, 2);
	Serial.print(", ");
/*	Serial.print(attitude.z_rate * RAD2DEG, 2);
	Serial.print(", ");
*/	Serial.print(attitude.pitch * RAD2DEG, 2);
	Serial.print(", ");
	Serial.print(attitude.roll * RAD2DEG, 2);
	Serial.print(", ");
/*	Serial.print(attitude.heading * RAD2DEG, 2);
	Serial.print(", ");
*/	Serial.print(attitude.rspeed * RAD2DEG, 2);
/*	Serial.print(", ");
	Serial.print(attitude.altitude);
	Serial.print(", ");
	Serial.print(attitude.vspeed);
	Serial.print(", ");
	Serial.print(attitude.temperature);
	Serial.print(", ");
	Serial.print(attitude.pressure);
*/
	Serial.println();
}

/* ----- Main loop ----- */
void loop()
{	
        unsigned long start = millis(), dt = start - timer;
        timer = start;

	// Get user commands
	if (xsr.readCal(&channels[0], &failSafe, &lostFrames)  && !failSafe) {
		// Translate throttle command from [-1;1] -> [0;1]
		//ESC_PULSE_SPEED_0_WIDTH + ((1.0 + throttle) / 2.0) * (ESC_PULSE_SPEED_FULL_WIDTH - ESC_PULSE_SPEED_0_WIDTH); 
		base_pulse = ESC_PULSE_SPEED_0_WIDTH + (1.0 + channels[CHNL_THROTTLE]) * speed_scale; 
	}
	
/*	armed = (channels[CHNL_ARMED] > 0);
	if (armed) 
		base_pulse = ESC_PULSE_SPEED_0_WIDTH;
	else
		base_pulse = ESC_PULSE_MIN_WIDTH;
*/		
/*
        // Change status from SAFE to STOP only if throttle is 0 and arming switch is off
	if (status == FLIGHT_STATUS_SAFE) {
		Serial.println("SAFE");
		if (!armed && throttle < 0.03) {
			status = FLIGHT_STATUS_STOP;
		} else{
			// Stay safe
			//Serial.println("Throttle lock safety");
		} 
		
		base_pulse = ESC_PULSE_MIN_WIDTH;
	}

        // Change status from STOP to ARMED only if throttle is 0 and arming switch is on
	if (status == FLIGHT_STATUS_STOP) {
		Serial.println("STOP");
		if (armed) {
			if (throttle < 0.02) {
				status = FLIGHT_STATUS_ARMED;
				base_pulse = ESC_PULSE_SPEED_0_WIDTH;
				Serial.println("Motors armed");
			} else {
				status = FLIGHT_STATUS_SAFE;
				base_pulse = ESC_PULSE_MIN_WIDTH;
			}
		} else {
			base_pulse = ESC_PULSE_MIN_WIDTH;
		}
	}
	
	if (status == FLIGHT_STATUS_ARMED) {
		if (!armed) {
			status = FLIGHT_STATUS_STOP;
			base_pulse = ESC_PULSE_MIN_WIDTH;
			Serial.println("Motors stop");
		} else {
			base_pulse = ESC_PULSE_SPEED_0_WIDTH + throttle * (ESC_PULSE_SPEED_FULL_WIDTH - ESC_PULSE_SPEED_0_WIDTH);
		}
	}
*/	
	// Refresh drone attitude
	get_attitude(dt);
	
        // Initialize all pulses w/ base pulse
        pulse_fr = pulse_fl = pulse_rr = pulse_rl = base_pulse;

	/* PID outputs reaction & pulses evolution
	 *  
	 * | When               | output sign is         | 
	 * -----------------------------------------------
	 * | roll right / left  | roll_output  < 0 / > 0 |
	 * | pitch front / rear | pitch_output < 0 / > 0 | 
	 * | yaw_right / left   | yaw_output   < 0 / > 0 |
	 */
	yaw_setpoint = 0; //channels[CHNL_YAW] * yaw_max_rate;
	yaw_output = pid_yaw.compute(attitude.rspeed, yaw_setpoint);

	if (mode == FLIGHT_MODE_LEVELED) {
		pitch_setpoint = 0; //channels[CHNL_PITCH] * pitch_max_angle;
	        roll_setpoint = 0; //channels[CHNL_ROLL] * roll_max_angle;

                pitch_output = pid_pitch.compute(attitude.pitch, pitch_setpoint);
                roll_output = pid_roll.compute(attitude.roll, roll_setpoint);

	} else if (mode == FLIGHT_MODE_ACRO) {
                pitch_setpoint = 0; //channels[CHNL_PITCH] * pitch_max_rate;
		roll_setpoint = 0; //channels[CHNL_ROLL] * roll_max_rate;

                pitch_output = pid_pitch.compute(attitude.y_rate, pitch_setpoint);
		roll_output = pid_roll.compute(attitude.x_rate, roll_setpoint);
	}
	        
	/* 
	 *  Adapt pulses of each motor using the following rules:
	 * 
	 *  pitch_output < 0 -> nose dips      -> front motors speed increases & rear motors speed decreases
	 *  pitch_output > 0 -> nose raises up -> front motors speed decreases & rear motors speed increases
	 *   roll_output < 0 -> rolls right    -> left motors speed increases & right motors speed decreases
	 *   roll_output > 0 -> rolls left     -> left motors speed decreases & right motors speed increases
	 *    yaw_output < 0 -> rotate right   -> front-right/rear-left motors speed increases & front-left/rear-right motors speed decreases
	 *    yaw_output > 0 -> rotate left    -> front-right/rear-left motors speed decreases & front-left/rear-right motors speed increases
	 */
	pulse_fr += - pitch_output + roll_output - yaw_output;
	pulse_rr += + pitch_output + roll_output + yaw_output;
	pulse_rl += + pitch_output - roll_output - yaw_output;
	pulse_fl += - pitch_output - roll_output + yaw_output;
/*
	Serial.print("yaw_out:");
	Serial.print(yaw_output);
	Serial.print(" pitch_out:");
	Serial.print(pitch_output);
	Serial.print(" roll_output:");
	Serial.println(roll_output);
*/	
/*
	Serial.print("fr:");
	Serial.print(pulse_fr);
	Serial.print(" fl:");
	Serial.print(pulse_fl);
	Serial.print(" rl:");
	Serial.print(pulse_rl);
	Serial.print(" rr:");
	Serial.println(pulse_rr);
*/	

        esc_FR.set_pulse(pulse_fr);
        esc_FL.set_pulse(pulse_fl);
        esc_RR.set_pulse(pulse_rr);
        esc_RL.set_pulse(pulse_rl);	

	led_update();
	delayMicroseconds(9500);
}
