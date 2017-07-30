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
const uint8_t MPU6050_INT = 6;

// SBUS channels mapping
// These pins are compatibles w/ either Servo or PWMServo libraries
// See https://www.pjrc.com/teensy/td_libs_Servo.html for details
const uint8_t SBUS_CHANNEL_THROTTLE = 0;
const uint8_t SBUS_CHANNEL_YAW = 1;
const uint8_t SBUS_CHANNEL_PITCH = 2;
const uint8_t SBUS_CHANNEL_ROLL = 3;
const uint8_t SBUS_CHANNEL_ARMED = 4;
const uint8_t SBUS_CHANNEL_BUZZER = 5;

/*
 * ESC & motors
 */

// Pin numbers for ESC signals 
const uint8_t ESC_PIN_FR = 9;  // CCW
const uint8_t ESC_PIN_FL = 10;  // CW
const uint8_t ESC_PIN_BR = 22; // CW
const uint8_t ESC_PIN_BL = 23; // CCW

// Min/max pulse width in microseconds for ESC
const float ESC_MIN_PULSE_WIDTH = 1000;
const float ESC_MAX_PULSE_WIDTH = 2000;

const float MOTORS_ARM_SPEED = 0.22;

#endif // raven_h
