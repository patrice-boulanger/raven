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
 * Use jrowberg I2Cdev library for MPU6050
 * https://github.com/jrowberg/i2cdevlib
 */ 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

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
unsigned long dt_loop = 0;

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

/* ----- Read quaternion from MPU ----- */

bool mpu_read_quat(Quaternion *q)
{
	while(!mpuInterrupt && fifoCount < packetSize);

	// Reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	
	// Get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// Check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// Reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("MPU FIFO overflow"));
		
		return false;
	} else if (mpuIntStatus & 0x02) {
		// Check for DMP data ready interrupt (this should happen frequently)
		
		// Wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();
		
		// Read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		// Track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
		
		mpu.dmpGetQuaternion(q, fifoBuffer);

		return true;
	}
}

/* ----- Read Yaw/Pitch/Roll angles (radians) from MPU ----- */

bool mpu_read_ypr(float *yaw, float *pitch, float *roll)
{
	// Orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	if (!mpu_read_quat(&q))
		return false;
	
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

	// Radians
	*yaw = ypr[0];
	*pitch = ypr[1];
	*roll = ypr[2];

	return true;
}

/* ----- Find accel. & gyro offsets for the MPU ----- */

void mpu_calibrate()
{
	float dw, dx, dy, dz, avg = 0, alpha = 0.96;
	Quaternion last, q;
	int count = 100; // prevent to break the loop before at least count samples
	
	last.w = last.x = last.y = last.z = 0;
	
	while(true) {
		if (mpu_read_quat(&q)) {
			dw = q.w - last.w;
			dx = q.x - last.x;
			dy = q.y - last.y;
			dz = q.z - last.z;

			avg = alpha * avg + (1.0 - alpha) * (dw + dx + dy + dz);
			
			last.w = q.w;
			last.x = q.x;
			last.y = q.y;
			last.z = q.z;

			if (cnt == 0 && avg < 0.000001)
				break;

			cnt --;
		}
	}	

	// Save to EEPROM
	off_ax = mpu.getXAccelOffset();
	off_ax = mpu.getYAccelOffset();
	off_ax = mpu.getZAccelOffset();
	off_gx = mpu.getXGyroOffset();
	off_gy = mpu.getYGyroOffset();
	off_gz = mpu.getZGyroOffset();
	
	// Set offsets	
	mpu.setXAccelOffset(off_ax);
	mpu.setYAccelOffset(off_ay);
	mpu.setZAccelOffset(off_az);
	mpu.setXGyroOffset(off_gx);
	mpu.setYGyroOffset(off_gy);
	mpu.setZGyroOffset(off_gz);
}

/* ----- MPU6050 Interrupt detection routine ----- */

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
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

	// Initialize MPU
	Serial.println(F("> Initializing MPU"));
	mpu.initialize();

	// Configure the MPU6050 to +/-16g and +/-2000º/s
	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

	pinMode(MPU6050_PIN_INT, INPUT);

	// Check connection w/ MPU
	if (!mpu.testConnection()) {
		Serial.println(F("  Connection to MPU6050 failed !"));
		while(true);
	}

	// Configure DMP
	Serial.println(F("  Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
	
	// Check MPU status
	if (devStatus == 0) {
		// Turn on the DMP, now that it's ready
		Serial.println(F("  Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// Enable interrupt detection
		Serial.print(F("  Enabling interrupt detection on PIN "));
		Serial.println(MPU6050_PIN_INT);
		
		attachInterrupt(digitalPinToInterrupt(MPU6050_PIN_INT), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("  DMP ready"));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		Serial.print(F("  FIFO packet size "));
		Serial.print(packetSize);
		Serial.println(F(" bytes"));
		
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print("  DMP Initialization failed, code ");
		Serial.println(devStatus);
		while(true);
	}

	Serial.println("Calibrate");
	mpu_calibrate();
	
	Serial.println(F("Ready"));
	
	// Initialize loop timer
	dt_loop = millis();
}

// Dump all SBUS channels to serial
void channels_dump()
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
	
/* ----- Main loop ----- */
void loop()
{
	unsigned long now = millis();
	float throttle = 0.0;
	
	// Get user commands
	if (xsr.readCal(&channels[0], &failSafe, &lostFrames)) {
		
		if (failSafe) {
			// Out of range, stop everything :-/
			// do something ...
			Serial.println(F("!!! OUT OF RANGE !!!"));				
			m_FR.set_speed(0);
			m_FL.set_speed(0);
			m_BR.set_speed(0);
			m_BL.set_speed(0);
			
			return;
		}				

		if (channels[CMD_ARMED_ID] >= 0.0) {
			// Translate throttle value between [0;1] rather than [-1;1]
			throttle = MOTORS_ARM_SPEED + (1.0 - MOTORS_ARM_SPEED) * (1.0 + channels[CMD_THROTTLE_ID]) * 0.5;
		} 
	}

	// Get attitude
	float yaw, pitch, roll;
	if (mpu_read_ypr(&yaw, &pitch, &roll)) {
		Serial.print("ypr\t");
    		Serial.print(yaw * 180/M_PI);
    		Serial.print("\t");
    		Serial.print(pitch * 180/M_PI);
    		Serial.print("\t");
    		Serial.println(roll * 180/M_PI);
	}
}
