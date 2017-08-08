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

/* --- MPU6050: accelerometer/gyroscope --- */
#include "MPU6050.h"
//#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
// Calibration offsets: computed w/ I2Cdev IMU_Zero sketch
int off_ax = -989, off_ay = -492, off_az = 1557;
int off_gx = 36, off_gy = 1, off_gz = 26;

/* --- HMC5883L: magnetometer --- */
#include "HMC5883L.h"
HMC5883L compass;

// Calibration offsets
float off_decl_deg = 0, off_decl_min = 57.96; // declinaison angle degrees/minutes for Paris
float off_decl = 0.0; // declinaison angle

/* --- BMP180: barometer --- */
#include "BMP085.h"
BMP085 barometer;

/* --- Flight Controller Data --- */

// Complementary filter coefficient
#define ALPHA                   0.96
// Number of samples for altitude averaging
#define ALT_SAMPLES		20

typedef struct {
	float x_acc, y_acc, z_acc;	// acceleration (m.s-2)
	float x_rate, y_rate, z_rate;	// angular speeds (rads.s-1)
	
	float pitch, roll;		// pitch/roll angles (rads)

	float heading, heading_p;	// current & previous heading (rads)
	float rspeed;			// angular speed (rads.s-1)

	float pressure;			// pressure (pa)
	float temperature;		// temperature (deg. C)
	
	float alt_samples[ALT_SAMPLES]; // Altitude samples
	int alt_idx;			// Current index in buffer
	
	float altitude, altitude_p;	// current & previous altitude (m)
	float vspeed;			// vertical speed (m.s-1)

} attitude_t;

// Drone attitude
attitude_t attitude;

// Flight controller status
int status;

// Flight mode
int mode;

// Throttle rate from user
float throttle = 0;
// Max. rates for acro mode (rads.s-1)
float yaw_max_rate, pitch_max_rate, roll_max_rate;
// Max. angles for self-level mode (rads)
float pitch_max_angle, roll_max_angle;

// Motor arming switch
bool armed = false;
// Buzzer switch
bool buzzer = false;
// Camera declinaison angle
float camera_angle = 0;

// Degrees/radians conversion
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

/* ----- Get drone attitude ----- */
void get_attitude(unsigned long ms)
{
	// Delta time in seconds
	float dt = ms / 1000.0;

        // Get raw values from MPU
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	
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
        attitude.roll = ALPHA * (attitude.roll + attitude.x_rate * dt) + (1 - ALPHA) * x_angle; 
        attitude.pitch = ALPHA * (attitude.pitch + attitude.y_rate * dt) + (1 - ALPHA) * y_angle; 

	// Backup previous heading
	attitude.heading_p = attitude.heading;

	int16_t mx, my, mz, temp;

	while(!compass.getReadyStatus());
	compass.getHeading(&mx, &my, &mz);

	if (mx == -4096 || mx == -4096 || mz == -4096) {
		Serial.println("Compass overflow");
		mx = my = mz = 0;
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
	
  	// Request temperature
    	barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    	// Wait appropriate time for conversion (4.5ms delay)
    	unsigned long lastMicros = micros();
    	while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    	// Read calibrated temperature value in degrees Celsius
    	attitude.temperature = barometer.getTemperatureC();

	// Request pressure (3x oversampling mode, high detail, 23.5ms delay)
    	barometer.setControl(BMP085_MODE_PRESSURE_3);
    	while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    	// Read calibrated pressure value in Pascals (Pa)
    	attitude.pressure = barometer.getPressure();

    	// Calculate absolute altitude in meters based on known pressure
    	attitude.altitude_p = attitude.altitude;  
	
	attitude.alt_samples[attitude.alt_idx] = barometer.getAltitude(attitude.pressure);
	attitude.alt_idx = (attitude.alt_idx + 1) % ALT_SAMPLES;

	attitude.altitude = 0;
	for(int i = 0; i < ALT_SAMPLES; i ++) 
		attitude.altitude += attitude.alt_samples[i];

	attitude.altitude /= ALT_SAMPLES;
	
	// Vertical speed
    	attitude.vspeed = (attitude.altitude - attitude.altitude_p) / dt;
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
	
	m_FR.set_pulse(front_right_speed);
	m_FL.set_pulse(front_left_speed);
	m_BR.set_pulse(back_right_speed);
	m_BL.set_pulse(back_left_speed);
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
	
  	// Configure
	compass.setGain(HMC5883L_GAIN_1090);
	compass.setMode(HMC5883L_MODE_CONTINUOUS);
	compass.setDataRate(HMC5883L_RATE_30);
	compass.setSampleAveraging(HMC5883L_AVERAGING_8);

	off_decl = (off_decl_deg + (off_decl_min / 60.0)) * (M_PI / 180.0); 
  	
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

        // Default values
        yaw_max_rate = (30 * DEG2RAD);
        pitch_max_rate = (15 * DEG2RAD);
        roll_max_rate = (15 * DEG2RAD);
        
        pitch_max_angle = (15 * DEG2RAD);
        roll_max_angle = (15 * DEG2RAD);

        mode = FLIGHT_MODE_LEVELED;

	Serial.println(F("Rock'n'roll"));

	// Initialize loop timer
	timer = millis();

	status = FLIGHT_STATUS_STOP;

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
	Serial.print(attitude.x_acc, 2);
	Serial.print(",");
	Serial.print(attitude.y_acc, 2);
	Serial.print(", ");
	Serial.print(attitude.z_acc, 2);
	Serial.print(", ");
	Serial.print(attitude.x_rate * RAD2DEG, 2);
	Serial.print(", ");
	Serial.print(attitude.y_rate * RAD2DEG, 2);
	Serial.print(", ");
	Serial.print(attitude.z_rate * RAD2DEG, 2);
	Serial.print(", ");
	Serial.print(attitude.pitch * RAD2DEG, 2);
	Serial.print(", ");
	Serial.print(attitude.roll * RAD2DEG, 2);
	Serial.print(", ");
	Serial.print(attitude.heading * RAD2DEG, 2);
	Serial.print(", ");
	Serial.print(attitude.rspeed * RAD2DEG, 2);
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
        unsigned long start = millis(), dt = start - timer;
        timer = start;

        // Pulse widths for each motor, base pulse is computed from throttle
        int16_t esc_fr, esc_fl, esc_br, esc_bl, base_pulse;
  
	// Get user commands
	if (!xsr.readCal(&channels[0], &failSafe, &lostFrames)) {
		// Not connected or short packet read
		return;
	}

        // Communication lost or out of range, stop everything :-/
	if (failSafe) {
	        Serial.println(F(" -!-!-!- OUT OF RANGE -!-!-!-"));				
		status = FLIGHT_STATUS_SAFE;
		base_pulse = ESC_PULSE_MIN_WIDTH;
	}  

	// Translate throttle command from [-1;1] -> [0;1]
	throttle = (1.0 + channels[CMD_THROTTLE_ID]) / 2; 
	// Arm switch
	armed = (channels[CMD_ARMED_ID] >= 0);

        // Change status from SAFE to STOP only if throttle is 0 and arming switch is off
	if (status == FLIGHT_STATUS_SAFE) {
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
		if (armed) {
			if (throttle < 0.03) {
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

        esc_fr = esc_fl = esc_br = esc_bl = base_pulse;

        // Refresh drone attitude
        get_attitude(dt);

        if (status == FLIGHT_STATUS_ARMED) {
                float yaw_setpoint, pitch_setpoint, roll_setpoint;

                yaw_setpoint = channels[CMD_YAW_ID] * yaw_max_rate;

                if (mode == FLIGHT_MODE_LEVELED) {
                        pitch_setpoint = channels[CMD_PITCH_ID] * pitch_max_angle;
                        roll_setpoint = channels[CMD_ROLL_ID] * roll_max_angle;
                } else if (mode == FLIGHT_MODE_ACRO) {
                        pitch_setpoint = channels[CMD_PITCH_ID] * pitch_max_rate;
                        roll_setpoint = channels[CMD_ROLL_ID] * roll_max_rate;
                }

                
        }

	//dump_attitude();

	m_FR.set_pulse(esc_fr);
	m_FL.set_pulse(esc_fl);
	m_BR.set_pulse(esc_br);
	m_BL.set_pulse(esc_bl);

/*
	Serial.print("dt =");
	Serial.print((float)(dt) / 1000, 3);
	Serial.println("ms");
*/
}
