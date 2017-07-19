
/*
 * raven.h - main raven include file
 */

#ifndef raven_h
#define raven_h

#include <Wire.h>

#if !defined(TEENSYDUINO)
#error This code runs on Teensy board >= 3.0
#endif 

#define RAVEN_VERSION "0.2"

// Arduino PINs
const uint8_t PIN_MPU_INT = 2;

// PWM PINs for motors
const uint8_t PIN_MOTOR_FR = 9;  // CCW
const uint8_t PIN_MOTOR_FL = 10;  // CW
const uint8_t PIN_MOTOR_BR = 22; // CW
const uint8_t PIN_MOTOR_BL = 23; // CCW

// Maximal PWM values in manual mode
const uint16_t MANUAL_MODE_PWM_MAX = 255;
const uint16_t MANUAL_MODE_PWM_SENSITIVITY_MAX = (MANUAL_MODE_PWM_MAX / 10);

// SBUS channels mapping
const uint8_t SBUS_CHANNEL_THROTTLE = 0;
const uint8_t SBUS_CHANNEL_ROLL = 1;
const uint8_t SBUS_CHANNEL_PITCH = 2;
const uint8_t SBUS_CHANNEL_YAW = 3;

#endif // raven_h
