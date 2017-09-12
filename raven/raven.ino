#include "raven.h"
#include "buzzer.h"
#include "led.h"
#include "motor.h"
#include "gimbal.h"
#include "console.h"
#include "state.h"
/*
 * br3ttb PID library
 * https://github.com/br3ttb/Arduino-PID-Library.git
 */
#include "PID_v1.h"
/* 
 * bolderflight SBUS library for Teensy 
 * https://github.com/bolderflight/SBUS.git
 */
#include "SBUS.h"

/*
 * Global state
 */
state_t state;

// FRSky XSR receiver in SBUS mode on Serial2
// https://www.pjrc.com/teensy/td_uart.html:
// Serial1 & 2 support 8 byte transmit and receive FIFOs, which allow for higher 
// speed baud rates, even when other libraries create interrupt latency.
SBUS xsr(Serial2);

float channels[16]; // SBUS calibrated data
uint8_t failSafe = 0;
uint16_t lostFrames = 0;

// Flight mode
int mode;

// Motor arming switch
bool armed = false;
// Camera angle setpoint
float camera_pitch = 0;
// Gimbal enable
bool gimbal_enable = false;
// Monitoring
bool monitoring = false;
unsigned long mon_timer;
char mon_string[80];

// Target angles computed from user inputs & max values
double yaw_setpoint, pitch_setpoint, roll_setpoint;
// PID controllers actions
double yaw_output, pitch_output, roll_output;

Motor esc_FR(ESC_PIN_FR); // Front-Right CCW
Motor esc_FL(ESC_PIN_FL); // Front-Left  CW
Motor esc_RR(ESC_PIN_RR); // Rear-Right  CW
Motor esc_RL(ESC_PIN_RL); // Rear-Left   CCW

// Pulse interval for throttle
// Divide by 2 to translate throttle from [-1;+1] to [0;1] 
const float pulse_scale = 0.5 * (ESC_PULSE_SPEED_FULL_WIDTH - ESC_PULSE_SPEED_0_WIDTH);
// Pulsation computed from throttle
int16_t pulse_throttle;
// Pulsation width adjustements for each motor, computed w/ PID outputs
int16_t pulse_fr, pulse_fl, pulse_rr, pulse_rl; 

// Main loop timer (µsec)
unsigned long timer = 0;

// Declare the PID at the end since they need to refer to previously declared variables
PID pid_yaw(&state.attitude.heading_rate, &yaw_output, &yaw_setpoint, 0, 0, 0, P_ON_E, DIRECT);
PID pid_roll(&state.attitude.roll, &roll_output, &roll_setpoint, 0, 0, 0, P_ON_E, DIRECT);
PID pid_pitch(&state.attitude.pitch, &pitch_output, &pitch_setpoint, 0, 0, 0, P_ON_E, DIRECT);

/* ----- Dead-end ----- */
void fail_forever()
{
	led_clear();
	delay(10);
	
	led_sequence("R__r______");

	// Stay here forever
	while(true) {
		led_update();
		delay(10);
	}
}

/* ----- Setup ----- */
void setup()
{
	// Clear attitude structures
	memset(&state.sma, 0, sizeof(sma_buffer_t));
	memset(&state.attitude, 0, sizeof(attitude_t));

	// Start serial console
	Serial.begin(115200);

        // Buzzer
        pinMode(BUZZER_PIN, OUTPUT);
        
	// LEDs
	pinMode(LED_GREEN_PIN, OUTPUT);
	pinMode(LED_RED_PIN, OUTPUT);
	pinMode(LED_WHITE_PIN, OUTPUT);

	led_clear();	

	// Fixed green led switched on during setup
	led_on(LED_GREEN_PIN);

	// Hello world ...
	Serial.println(F("\r\n---------------------"));
	Serial.print(F("raven v"));
	Serial.print(RAVEN_VERSION);
	Serial.println(F(", hello world"));

	// Load configuration from EEPROM
	Serial.print(F("EEPROM: ")); 
	if (!eeprom_load(&state.config)) {
		Serial.println(F("invalid checksum, reset to default"));
		eeprom_reset(&state.config);
	} else {
		Serial.print(sizeof(eeprom_config_t));
		Serial.println(F(" bytes loaded"));
	}
	
	// Join I2C bus (I2Cdev library doesn't do this automatically)
	Serial.print(F("Starting I2C bus "));
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock

	for(int i = 0; i < 5; i ++) {
		Serial.print(".");
		delay(400);
	}
	
	Serial.println(" ok");

	Serial.print("Init. & test devices: ");
	
	// Initialize compass
	Serial.print("compass ");
	state.compass.initialize();
	
	// Check connection w/ compass
	if (!state.compass.testConnection()) {
		Serial.println(F("failed"));
		fail_forever();
	} else
		Serial.print(F("ok,"));
	
  	// Configure
	state.compass.setGain(HMC5883L_GAIN_1090);
	state.compass.setMode(HMC5883L_MODE_CONTINUOUS);
	state.compass.setDataRate(HMC5883L_RATE_30);
	state.compass.setSampleAveraging(HMC5883L_AVERAGING_8);

	// Initialize barometer
	Serial.print(" barometer ");
//	state.barometer.initialize();
	// Datasheet: startup time after power-up the device: 10ms
//	delay(10);
	
	// Check connection w/ barometer
	if (!state.barometer.testConnection()) {
		Serial.println(F("failed"));
//		fail_forever();
	} else 
		Serial.print(F("ok,"));

	// Initialize MPU
	Serial.print(F(" MPU "));
	state.mpu.initialize();
	// Datasheet: gyroscope start-up time: 30ms
	delay(30);
	
	// Check connection w/ MPU
	if (!state.mpu.testConnection()) {
		Serial.println(F("failed"));
		fail_forever();
	} else 
		Serial.println(F("ok"));

        // Configure
	state.mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // +/- 250 degrees/sec 
  	state.mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // +/- 2g

	state.mpu.setXAccelOffset(state.config.off_ax);
	state.mpu.setYAccelOffset(state.config.off_ay);
	state.mpu.setZAccelOffset(state.config.off_az);

	state.mpu.setXGyroOffset(state.config.off_gx);
	state.mpu.setYGyroOffset(state.config.off_gy);
	state.mpu.setZGyroOffset(state.config.off_gz);

	// Enable Digital Low Pass Filter on the MPU against motors vibrations
	state.mpu.setDLPFMode(MPU6050_DLPF_BW_5);
	
	// Gimbal settings
	Serial.print(F("Gimbal dance ... "));
	gimbal_init();
	//gimbal_dance();
	Serial.println(F("done"));
	
        // Configure flight controller
	pid_yaw.SetOutputLimits(state.config.yaw_pid_min, state.config.yaw_pid_max);
	pid_pitch.SetOutputLimits(state.config.pitch_pid_min, state.config.pitch_pid_max);
	pid_roll.SetOutputLimits(state.config.roll_pid_min, state.config.roll_pid_max);

        // Call SetSampleTime() before SetTunings() because the default sample time set in the constructor (100ms) 
        // will be used to compute the values of the Ki/Kd coefficients, which will be 10 times different than their values!
        // See PID_v1.cpp lines 115-137 & lines 150-158.
        pid_yaw.SetSampleTime(state.config.pid_sample);
        pid_pitch.SetSampleTime(state.config.pid_sample);
        pid_roll.SetSampleTime(state.config.pid_sample);

	pid_yaw.SetTunings(state.config.yaw_kp, state.config.yaw_ki, state.config.yaw_kd);
	pid_pitch.SetTunings(state.config.pitch_kp, state.config.pitch_ki, state.config.pitch_kd);
	pid_roll.SetTunings(state.config.roll_kp, state.config.roll_ki, state.config.roll_kd);

        mode = FLIGHT_MODE_AUTO_LEVEL;

	// Blinking green led while waiting for the transmitter
	led_sequence("G___g________________");

	Serial.println(F("Enable SBUS receiver"));
	xsr.begin();
	
	Serial.println(F("Waiting for TX connection"));
	Serial.println(F("Switch on the TX with full pitch/roll to start the Bluetooth console"));
	
        int cnt = 10, full_pitch_roll = 0, minus_pitch_roll = 0;
        while(cnt != 0) {
		if (xsr.readCal(&channels[0], &failSafe, &lostFrames) && !failSafe) {
			cnt --;

			if (channels[CHNL_PITCH] > 0.98 && channels[CHNL_ROLL] > 0.98)
				full_pitch_roll ++;
			else if (channels[CHNL_PITCH] < -0.98 && channels[CHNL_ROLL] < -0.98)
				minus_pitch_roll ++;
		}

  		led_update();   
		delay(10);
	}

	if (full_pitch_roll == 10) {
		led_clear();

		buzzer_tone(440, 100);
		delay(10);
		buzzer_tone(440, 100);
		delay(10);
		buzzer_tone(440, 100);
		delay(10);

		// Starts console on Serial3 == BT
		Serial3.begin(57600);
		start_console(Serial3, &state);
	} else if (minus_pitch_roll == 10) {
		buzzer_tone(880, 500);
		delay(10);

		// Starts monitoring on Serial3 == BT
		Serial3.begin(57600);

		monitoring= true;
		mon_timer = millis();
		
	} else {
		buzzer_tone(1760, 100);
		delay(10);
		buzzer_tone(1760, 100);
		delay(10);
		buzzer_tone(1760, 100);
		delay(10);
	
		Serial.println(F("rock'n'roll"));
	}
	
	// Fly mode 
	led_sequence("G_______________R_______________W_______________g_______r_______");

        timer = micros();

        // Safety delay
	delay(9);
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
	Serial.print(state.attitude.roll_rate, 2);
	Serial.print(", ");
	Serial.print(state.attitude.pitch_rate, 2);
	Serial.print(", ");
	Serial.print(state.attitude.yaw_rate, 2);
	Serial.print(", ");
	Serial.print(state.attitude.pitch);
	Serial.print(", ");
	Serial.print(state.attitude.roll);
	Serial.print(", ");
	Serial.print(state.attitude.heading, 2);
	Serial.print(", ");
	Serial.print(state.attitude.heading_rate);
	
	Serial.println("");
}

/* ----- Main loop ----- */
void loop()
{	
	bool armed_prev = armed;
	
	memset(channels, 0, sizeof(float));
	
        unsigned long now = micros(), dt = now - timer;

	// Get user commands
	if (xsr.readCal(channels, &failSafe, &lostFrames)) {		
		if (channels[CHNL_BUZZER] > 0.9) {
			tone(BUZZER_PIN, 1320, 100);
		}

		gimbal_enable = (channels[CHNL_GIMBAL] > 0.9);
		 
		// channel[-1; 1] -> camera_pitch[0;180]
		camera_pitch = 90 * (1 + channels[CHNL_CAMERA]);

		armed = (channels[CHNL_ARMED] > 0.0);
		if (armed) {
			if (armed != armed_prev) {
				// Activate PID
				pid_yaw.SetMode(AUTOMATIC);
				pid_pitch.SetMode(AUTOMATIC);
				pid_roll.SetMode(AUTOMATIC);
			}
			
			// Translate throttle command from [-1;1] -> [0;1]
			//ESC_PULSE_SPEED_0_WIDTH + ((1.0 + throttle) / 2.0) * (ESC_PULSE_SPEED_FULL_WIDTH - ESC_PULSE_SPEED_0_WIDTH); 
			pulse_throttle = ESC_PULSE_SPEED_0_WIDTH + (1.0 + channels[CHNL_THROTTLE]) * pulse_scale; 
		} else {
			if (armed != armed_prev) {
				// Disable PID			
				pid_yaw.SetMode(MANUAL);
				pid_pitch.SetMode(MANUAL);
				pid_roll.SetMode(MANUAL);
			}
					
			pulse_throttle = ESC_PULSE_MIN_WIDTH;
		}
	}

	// Refresh drone attitude
	get_raw_value(&state);	
	update_attitude(&state, dt);
	
	
	/* PID outputs reaction & pulses evolution
	 *  
	 * | When               | output sign is         | 
	 * -----------------------------------------------
	 * | roll right / left  | roll_output  < 0 / > 0 |
	 * | pitch front / rear | pitch_output < 0 / > 0 | 
	 * | yaw_right / left   | yaw_output   < 0 / > 0 |
	 */
	yaw_setpoint = 0;
	pitch_setpoint = 0;
	roll_setpoint = 0;

	// Get the setpoints according the user command, filter inputs from noise around zero
	if (abs(channels[CHNL_YAW]) > 0.02)
		yaw_setpoint = channels[CHNL_YAW] * state.config.yaw_max_rate;

	if (mode == FLIGHT_MODE_AUTO_LEVEL) {
		if (abs(channels[CHNL_PITCH]) > 0.02) 
			pitch_setpoint = round(channels[CHNL_PITCH] * state.config.pitch_max_angle);
		
	        if (abs(channels[CHNL_ROLL]) > 0.02) 	
	        	roll_setpoint = round(channels[CHNL_ROLL] * state.config.roll_max_angle);
			
	} else if (mode == FLIGHT_MODE_ACRO) {
		if (abs(channels[CHNL_PITCH]) > 0.02) 
			pitch_setpoint = channels[CHNL_PITCH] * state.config.pitch_max_rate;

	        if (abs(channels[CHNL_ROLL]) > 0.02) 	
			roll_setpoint = channels[CHNL_ROLL] * state.config.roll_max_rate;
	}

	// Compute PID compensation
	pid_yaw.Compute();
	pid_pitch.Compute();
	pid_roll.Compute();

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
	
        // Initialize all pulses w/ base pulse
        pulse_fr = pulse_fl = 1.02 * pulse_throttle;
        pulse_rr = pulse_rl = pulse_throttle;
        
/*	// Adjust w/ PID outputs if motors are armed
	if (armed) {
		pulse_fr += - (int)(pitch_output) - (int)(roll_output); // - yaw_output;
		pulse_rr += + (int)(pitch_output) - (int)(roll_output); // + yaw_output;
		pulse_rl += + (int)(pitch_output) + (int)(roll_output); // - yaw_output;
		pulse_fl += - (int)(pitch_output) + (int)(roll_output); // + yaw_output;
	}
*/	
	// Set the pulses for each motor
        esc_FR.set_pulse(pulse_fr);
        esc_FL.set_pulse(pulse_fl);
        esc_RR.set_pulse(pulse_rr);
        esc_RL.set_pulse(pulse_rl);	

	// If gimbal mode enabled, compensate the pitch angle of the drone to fix camera angle
	gimbal_set_pitch(gimbal_enable ? (camera_pitch - state.attitude.pitch) : camera_pitch);
	
	led_update();

	if (monitoring && millis() - mon_timer > 200) {
		snprintf(mon_string, 80, "%.1f,%.1f,%.1f,%.1f,%.1f", //,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
				state.attitude.pitch,state.attitude.pitch_rate, //pitch_setpoint, pitch_output,
				state.attitude.roll, state.attitude.roll_rate, //roll_setpoint, roll_output,
				//state.attitude.yaw_rate, 
				state.attitude.heading); //, yaw_setpoint, yaw_output);

		Serial3.println(mon_string);
		
		mon_timer = millis();
	}

	// Loop duration at least 9ms
	while((micros() - now) < 9000);
        timer = now;	
}
