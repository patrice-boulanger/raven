#ifndef _RAVEN_CONSOLE_H
#define _RAVEN_CONSOLE_H

#include <Arduino.h>
#include "state.h"

/* 
 * Start the console on the specified serial port
 */
void start_console(HardwareSerial &serial, state_t *state);

#endif // _RAVEN_CONSOLE_H
