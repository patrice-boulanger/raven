/*
 * raven.h - main raven include file
 */

#ifndef raven_h
#define raven_h

#include <Wire.h>

#if !defined(TEENSYDUINO)
#error This code runs on Teensy board >= 3.0
#endif 

#define RAVEN_VERSION "0.2.2"

// LED PINS
const uint8_t LED_GREEN_PIN = 5;
const uint8_t LED_RED_PIN = 4;
const uint8_t LED_WHITE_PIN = 3;

// Buzzer
const uint8_t BUZZER_PIN = 6;

// User commands index
const uint8_t CMD_THROTTLE_ID = 0;
const uint8_t CMD_YAW_ID = 1;
const uint8_t CMD_PITCH_ID = 2;
const uint8_t CMD_ROLL_ID = 3;
const uint8_t CMD_ARMED_ID = 4;
const uint8_t CMD_BUZZER_ID = 5;
const uint8_t CMD_CAMERA_ID = 6;

/* 
 * Pin numbers for ESC signals
 * These pins are compatibles w/ either Servo or PWMServo libraries
 * See https://www.pjrc.com/teensy/td_libs_Servo.html for details
 * */
const uint8_t ESC_PIN_FR = 20;  // CCW
const uint8_t ESC_PIN_FL = 21;  // CW
const uint8_t ESC_PIN_RR = 22; // CW
const uint8_t ESC_PIN_RL = 23; // CCW

/* 
 * Pulses width in microseconds for ESC
 *
 */
// Minimal pulse width 
const int ESC_PULSE_MIN_WIDTH = 1000;
// Pulse for minimal motor speed (motors armed)
const int ESC_PULSE_SPEED_0_WIDTH = 1200;
// Pulse at full speed (keep some room for attitude correction)
const int ESC_PULSE_SPEED_FULL_WIDTH = 1800;
// Maximal pulse width
const int ESC_PULSE_MAX_WIDTH = 2000;
// Pulse for middle point
const int ESC_PULSE_MIDDLE = 1500;

/*
 * Flight controller status
 */
// Motors security mode, everything stopped
const uint8_t FLIGHT_STATUS_SAFE = 0;
// Motors stopped
const uint8_t FLIGHT_STATUS_STOP = 1;
// Alarm if throttle != 0 or motors armed
const uint8_t FLIGHT_STATUS_ALARM = 2;
// Motors running, ready to fly
const uint8_t FLIGHT_STATUS_ARMED = 3;

/*
 * Flight modes
 */
const uint8_t FLIGHT_MODE_LEVELED = 0;
const uint8_t FLIGHT_MODE_ACRO = 1;
#endif // raven_h
