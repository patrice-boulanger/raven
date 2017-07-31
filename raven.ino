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

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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

// Command as signed percents
float throttle;
float yaw;
float pitch;
float roll;	
bool armed;
bool buzzer;

// Loop timer
unsigned long dt_loop = 0;

/* ----- MPU6050 Interrupt detection routine ----- */

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
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
void set_motor_speed_manual()
{
	// Keep some room for motors adjustement
	if (throttle > 0.9)
		throttle = 0.9;

	float front_right_speed, front_left_speed, back_right_speed, back_left_speed;
	front_right_speed = front_left_speed = back_right_speed = back_left_speed = throttle;
	
	front_right_speed  += 0.15 * (- roll - pitch - yaw) / 3;
	back_left_speed    += 0.15 * (  roll + pitch - yaw) / 3;
	front_left_speed   += 0.15 * (  roll - pitch + yaw) / 3;
	back_right_speed   += 0.15 * (- roll + pitch + yaw) / 3;
}

// Setup
void setup()
{
	// Start serial console
	Serial.begin(9600);

	delay(200);
	
	Serial.print(F("raven v"));
	Serial.print(RAVEN_VERSION);
	Serial.println(F(" starting"));

	// Join I2C bus (I2Cdev library doesn't do this automatically)
	Serial.println(F("- Initializing I2C bus"));
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

	// Initialize MPU
	Serial.println(F("- Initializing MPU"));
	mpu.initialize();
	pinMode(MPU6050_PIN_INT, INPUT);

	// Check connection w/ MPU
	if (!mpu.testConnection()) {
		Serial.println(F("  Connection to MPU6050 failed !"));
		while(true);
	}

	// Configure DMP
	Serial.println(F("  Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// TODO: calibration procedure
	mpu.setXGyroOffset(0);
	mpu.setYGyroOffset(0);
	mpu.setZGyroOffset(0);
	mpu.setZAccelOffset(0); // 1688 factory default for my test chip

	// Check MPU status
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("  Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
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
	
	Serial.println(F("- Initializing SBUS"));
	xsr.begin();
	
	Serial.println(F("Ready"));
	delay(500);
	
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

// Main loop
void loop()
{
	do {
		// Get user commands
		if (xsr.readCal(&channels[0], &failSafe, &lostFrames)) {
			// Out of range, stop everything :-/
			if (failSafe) {
				// do something ...
				Serial.println(F("!!! OUT OF RANGE !!!"));
				
				m_FR.set_speed(0);
				m_FL.set_speed(0);
				m_BR.set_speed(0);
				m_BL.set_speed(0);
				return;
			}
			
			buzzer = (channels[SBUS_CHANNEL_BUZZER] < 0.0);
			
			armed = (channels[SBUS_CHANNEL_ARMED] >= 0.0);
			if (!armed) {
				throttle = pitch = roll = yaw = 0.0;		
			} else {			
				throttle = (1.0 + channels[SBUS_CHANNEL_THROTTLE]) / 2.0; // 0 -> 1
				
				if (throttle > MOTORS_ARM_SPEED) {
					pitch = channels[SBUS_CHANNEL_PITCH]; // -1 -> 1
					roll = channels[SBUS_CHANNEL_ROLL];   // -1 -> 1
					yaw = channels[SBUS_CHANNEL_YAW];     // -1 -> 1
				} else
					throttle = MOTORS_ARM_SPEED;
			}
		} 
	} while(!mpuInterrupt && fifoCount < packetSize);


	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("MPU FIFO overflow !"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
        
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetEuler(euler, &q);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

		Serial.print("quat\t");
		Serial.print(q.w);
		Serial.print("\t");
		Serial.print(q.x);
		Serial.print("\t");
		Serial.print(q.y);
		Serial.print("\t");
		Serial.println(q.z);
	
		Serial.print("euler\t");
		Serial.print(euler[0] * 180/M_PI);
		Serial.print("\t");
		Serial.print(euler[1] * 180/M_PI);
		Serial.print("\t");
		Serial.println(euler[2] * 180/M_PI);

		Serial.print("ypr\t");
		Serial.print(ypr[0] * 180/M_PI);
		Serial.print("\t");
		Serial.print(ypr[1] * 180/M_PI);
		Serial.print("\t");
		Serial.println(ypr[2] * 180/M_PI);
		
		Serial.print("areal\t");
		Serial.print(aaReal.x);
		Serial.print("\t");
		Serial.print(aaReal.y);
		Serial.print("\t");
		Serial.println(aaReal.z);

		Serial.print("aworld\t");
		Serial.print(aaWorld.x);
		Serial.print("\t");
		Serial.print(aaWorld.y);
		Serial.print("\t");
		Serial.println(aaWorld.z);
	}

	
}
