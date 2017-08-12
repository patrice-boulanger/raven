#include "raven.h"
#include "buzzer.h"
#include "led.h"
#include "motor.h"
#include "PID_v1.h"
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

// FRSky XSR receiver in SBUS mode on Serial2
// https://www.pjrc.com/teensy/td_uart.html:
// Serial1 & 2 support 8 byte transmit and receive FIFOs, which allow for higher speed baud rates, even when other libraries create interrupt latency.
SBUS xsr(Serial2);
// SBUS channels & data
float channels[16];
uint8_t failSafe = 0;
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
	
	double pitch, roll;		// pitch/roll angles (rads)

	int16_t mx, my, mz;
	
	double heading, heading_p;	// current & previous heading (rads)
	double rspeed;			// angular speed (rads.s-1)

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

double yaw_setpoint, pitch_setpoint, roll_setpoint;
double yaw_output, pitch_output, roll_output;

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

// Declare the PID at the end since they need to point to the variables
PID pid_yaw(&attitude.rspeed, &yaw_output, &yaw_setpoint, 0.0, 0.0, 0.0, DIRECT);
PID pid_roll(&attitude.roll, &roll_output, &roll_setpoint, 0, 0, 0, DIRECT);
PID pid_pitch(&attitude.pitch, &pitch_output, &pitch_setpoint, 0, 0, 0, DIRECT);

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
	/*
  	// Request temperature
    	barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    	// Wait appropriate time for conversion (4.5ms delay)
    	unsigned long lastMicros = micros();
    	while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    	// Read calibrated temperature value (deg. Celsius)
    	attitude.temperature = barometer.getTemperatureC();

	// Request pressure (3x oversampling mode, high detail, 23.5ms delay)
    	barometer.setControl(BMP085_MODE_PRESSURE_0);
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
    	attitude.vspeed = (attitude.altitude - attitude.altitude_p) / dt;*/
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
		//while(true);
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
	// Datasheet: startup time after power-up the device: 10ms
	delay(10);
	
	// Check connection w/ barometer
	if (!barometer.testConnection()) {
		Serial.println(F("  Connection to BMP180 failed !"));
		//while(true);
	}

	// Initialize MPU
	Serial.println(F("> Initializing MPU"));
	mpu.initialize();
	// Datasheet: gyroscope start-up time: 30ms
	delay(30);
	
	// Check connection w/ MPU
	if (!mpu.testConnection()) {
		Serial.println(F("  Connection to MPU6050 failed !"));
		//while(true);
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
        yaw_max_rate = (30 * DEG2RAD);
        pitch_max_rate = (15 * DEG2RAD);
        roll_max_rate = (15 * DEG2RAD);
        
        pitch_max_angle = (15 * DEG2RAD);
        roll_max_angle = (15 * DEG2RAD);

	pid_yaw.SetOutputLimits(0, 0);
	pid_pitch.SetOutputLimits(-100, 100);
	pid_roll.SetOutputLimits(-100, 100);

	pid_yaw.SetTunings(0, 0, 0);
	pid_pitch.SetTunings(100, 0, 0);
	pid_roll.SetTunings(100, 0, 0);

	//pid_yaw.SetMode(AUTOMATIC);
	pid_pitch.SetMode(AUTOMATIC);
	pid_roll.SetMode(AUTOMATIC);
	
/*      
        pid_yaw.set_kpid(10, 0, 0.0);
        pid_yaw.set_minmax(-20, 20);
        
        pid_pitch.set_kpid(200, 50, 0.0);
        pid_pitch.set_minmax(-200, 200);
        
        pid_roll.set_kpid(200, 50, 0.0);
        pid_roll.set_minmax(-200, 200);
*/

        mode = FLIGHT_MODE_LEVELED;

	led_sequence("G______________g______");
	
	Serial.print(F("Waiting for TX ... "));
	
        int cnt = 10;
        while(cnt != 0) {
		if (xsr.readCal(&channels[0], &failSafe, &lostFrames) && !failSafe)
                        cnt --;

  		led_update();   
		delay(10);
	}

	Serial.println("OK");

	led_sequence("G_______________R_______________g_______r_______");

        timer = millis();
        // Safety delay
	delay(20);
}

// Dump all SBUS channels to serial
void dump_channels()
{
		for(int i = 0; i < 7; i ++) {
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
//	memset(channels, 16, sizeof(float));
	
        unsigned long start = millis(), dt = start - timer;
        timer = start;

	// Get user commands
	if (xsr.readCal(channels, &failSafe, &lostFrames)) {
		if (channels[CHNL_ARMED] > 0.0) {
			// Translate throttle command from [-1;1] -> [0;1]
			//ESC_PULSE_SPEED_0_WIDTH + ((1.0 + throttle) / 2.0) * (ESC_PULSE_SPEED_FULL_WIDTH - ESC_PULSE_SPEED_0_WIDTH); 
			base_pulse = ESC_PULSE_SPEED_0_WIDTH + (1.0 + channels[CHNL_THROTTLE]) * speed_scale; 
		} else
			base_pulse = ESC_PULSE_MIN_WIDTH;


		if (channels[CHNL_BUZZER] > 0.9) {
			tone(BUZZER_PIN, 1320, 100);
		}
	}

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
	pid_yaw.Compute();

	if (mode == FLIGHT_MODE_LEVELED) {
		pitch_setpoint = channels[CHNL_PITCH] * pitch_max_angle;
	        roll_setpoint = channels[CHNL_ROLL] * roll_max_angle;

		pid_pitch.Compute();
		pid_roll.Compute();
	} else if (mode == FLIGHT_MODE_ACRO) {
                pitch_setpoint = channels[CHNL_PITCH] * pitch_max_rate;
		roll_setpoint = channels[CHNL_ROLL] * roll_max_rate;

		pid_pitch.Compute();
		pid_roll.Compute();
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
	if (channels[CHNL_ARMED] > 0.0) {
		pulse_fr += - pitch_output - roll_output; // - yaw_output;
		pulse_rr += + pitch_output - roll_output; // + yaw_output;
		pulse_rl += + pitch_output + roll_output; // - yaw_output;
		pulse_fl += - pitch_output + roll_output; // + yaw_output;
	}

        esc_FR.set_pulse(pulse_fr);
        esc_FL.set_pulse(pulse_fl);
        esc_RR.set_pulse(pulse_rr);
        esc_RL.set_pulse(pulse_rl);	

	Serial.print("pitch setpoint/output:");
	Serial.print(pitch_setpoint);
	Serial.print("/");
	Serial.print(pitch_output);
	Serial.print(" roll setpoint/output:");
	Serial.print(roll_setpoint);
	Serial.print("/");
	Serial.print(roll_output);
	Serial.print(" -- fr:");
	Serial.print(pulse_fr);
	Serial.print(" fl:");
	Serial.print(pulse_fl);
	Serial.print(" rl:");
	Serial.print(pulse_rl);
	Serial.print(" rr:");
	Serial.println(pulse_rr);

	led_update();
}
