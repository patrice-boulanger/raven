/*
 * raven.h - main raven include file
 */

#ifndef raven_h
#define raven_h

#include <Wire.h>

#define RAVEN_VERSION "0.2"

// Arduino PINs
const uint8_t PIN_MPU_INT = 2;

const uint8_t PIN_CPPM_SIG = 8;
const uint8_t PIN_CPPM_PWR = 12;

const uint8_t PIN_MOTOR_FR = 3;  // CCW
const uint8_t PIN_MOTOR_FL = 5;  // CW
const uint8_t PIN_MOTOR_BR = 6; // CW
const uint8_t PIN_MOTOR_BL = 11; // CCW

const uint8_t PIN_LED_0 = A0;
const uint8_t PIN_LED_1 = A1;

const uint8_t PIN_BATTERY = A2;

// Maximal PWM values in manual mode
const uint16_t MANUAL_MODE_PWM_MAX = 255;
const uint16_t MANUAL_MODE_PWM_SENSITIVITY_MAX = (MANUAL_MODE_PWM_MAX / 5);

// CPPM range
const int16_t CPPM_MIN_VALUE = -2;
const int16_t CPPM_MAX_VALUE = 258;
const uint16_t CPPM_RANGE = CPPM_MAX_VALUE - CPPM_MIN_VALUE;

// CPPM channels
const uint8_t CPPM_CHANNELS = 8;
const uint8_t CPPM_THROTTLE = 0;
const uint8_t CPPM_ROLL = 1;
const uint8_t CPPM_PITCH = 2;
const uint8_t CPPM_YAW = 3;

#endif // raven_h
