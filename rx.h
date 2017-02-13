#ifndef _RAVEN_RX_H
#define _RAVEN_RX_H

// Iniitialize SBUS interface
void RX_init();
// Get data from RX 
void RX_get_command(float *throttle, float *yaw, float *pitch, float *roll);


#endif // _RAVEN_RX_H
