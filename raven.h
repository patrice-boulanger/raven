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

// MPU6050 Interrupt PIN
const uint8_t MPU6050_PIN_INT = 6;

// User commands index
const uint8_t CMD_THROTTLE_ID = 0;
const uint8_t CMD_YAW_ID = 1;
const uint8_t CMD_PITCH_ID = 2;
const uint8_t CMD_ROLL_ID = 3;
const uint8_t CMD_ARMED_ID = 4;
const uint8_t CMD_BUZZER_ID = 5;

/*
 * ESC & motors
 */

// Pin numbers for ESC signals
// These pins are compatibles w/ either Servo or PWMServo libraries
// See https://www.pjrc.com/teensy/td_libs_Servo.html for details
const uint8_t ESC_PIN_FR = 9;  // CCW
const uint8_t ESC_PIN_FL = 10;  // CW
const uint8_t ESC_PIN_BR = 22; // CW
const uint8_t ESC_PIN_BL = 23; // CCW

// Pulses width in microseconds for ESC
const float ESC_PULSE_MIN_WIDTH = 1000;
const float ESC_PULSE_MAX_WIDTH = 2000;

// Throttle offset (%) when motors are armed [0;1]
const float MOTOR_ARMED_SPEED = 0.2;

#endif // raven_h
