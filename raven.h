
/*
 * raven.h - main raven include file
 */

#ifndef raven_h
#define raven_h

#include <Wire.h>

#if !defined(TEENSYDUINO)
#error This code runs on Teensy board >= 3.0
#endif 

#define RAVEN_VERSION "0.2.1"

// Arduino PINs
const uint8_t PIN_MPU_INT = 2;

// PWM PINs for motors
const uint8_t PIN_MOTOR_FR = 9;  // CCW
const uint8_t PIN_MOTOR_FL = 10;  // CW
const uint8_t PIN_MOTOR_BR = 22; // CW
const uint8_t PIN_MOTOR_BL = 23; // CCW

// SBUS channels mapping
// These pins are compatibles w/ either Servo or PWMServo libraries
// See https://www.pjrc.com/teensy/td_libs_Servo.html for details
const uint8_t SBUS_CHANNEL_THROTTLE = 0;
const uint8_t SBUS_CHANNEL_ROLL = 1;
const uint8_t SBUS_CHANNEL_PITCH = 2;
const uint8_t SBUS_CHANNEL_YAW = 3;

// Motor control
const uint16_t MOTOR_MIN_SPEED = 0;
const uint16_t MOTOR_MAX_SPEED = 180;
const uint16_t MOTOR_DELTA_SPEED = (MOTOR_MAX_SPEED - MOTOR_MIN_SPEED);
const float MOTOR_SPEED_REACTIVTY = 0.05; // percent

#endif // raven_h
