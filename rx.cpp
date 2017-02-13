//#include "SBUS.h"
#include "rx.h"

//SBUS sbus(Serial);

void RX_init()
{
//	sbus.begin(false);
}


void RX_get_command(float *throttle, float *yaw, float *pitch, float *roll)
{
/*
	sbus.process();

# warning mapping between channels & commands
	
	*throttle = sbus.getNormalizedChannel(1);
	*yaw = sbus.getNormalizedChannel(2);
	*pitch = sbus.getNormalizedChannel(3);
	*roll = sbus.getNormalizedChannel(4);		
*/
	*throttle = 0;
	*yaw = 0;
	*pitch = 0;
	*roll = 0;
}
