#ifndef _RAVEN_MOTOR_H
#define _RAVEN_MOTOR_H

// Motors ID
#define MOTOR_FRONT_RIGHT 0  // CCW
#define MOTOR_FRONT_LEFT  1  // CW
#define MOTOR_BACK_LEFT   2  // CCW
#define MOTOR_BACK_RIGHT  3  // CW

// Define some limits for speed of the motors
#define MOTOR_SPEED_STOP  0
#define MOTOR_SPEED_MIN   50
#define MOTOR_SPEED_MAX   255

// Set the digital PINs for each motor
void motor_set_pin(uint8_t front_right_pin, uint8_t front_left_pin, uint8_t back_left_pin, uint8_t back_right_pin);

// Set motors speed in one time
void motor_set_speed(uint16_t front_right, uint16_t front_left, uint16_t back_left, uint16_t back_right);

#endif // _RAVEN_MOTOR_H
