/*
 * raven.h - main raven include file
 */

#ifndef _RAVEN_H
#define _RAVEN_H

#include "Arduino.h"

#if !defined(TEENSYDUINO)
#error This code runs on Teensy board >= 3.0
#endif 

#define RAVEN_VERSION "0.2.3"

// LED PINS
const uint8_t LED_GREEN_PIN = 5; // green
const uint8_t LED_RED_PIN = 4;   // orange
const uint8_t LED_WHITE_PIN = 3; // white

// Buzzer
const uint8_t BUZZER_PIN = 6; // blue

// User commands index
const uint8_t CHNL_THROTTLE = 0;
const uint8_t CHNL_ROLL = 1;
const uint8_t CHNL_PITCH = 2;
const uint8_t CHNL_YAW = 3;
const uint8_t CHNL_ARMED = 4;
const uint8_t CHNL_BUZZER = 5;
const uint8_t CHNL_CAMERA = 6;

/* 
 * Pin numbers for ESC signals
 * These pins are compatibles w/ either Servo or PWMServo libraries
 * See https://www.pjrc.com/teensy/td_libs_Servo.html for details
 * */
const uint8_t ESC_PIN_FR = 20; // white
const uint8_t ESC_PIN_FL = 21; // violet
const uint8_t ESC_PIN_RR = 22; // blue
const uint8_t ESC_PIN_RL = 23; // grey

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

#endif // _RAVEN_H
