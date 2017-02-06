/*
 * Digital compas HMC5883L
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "hmc5883l.h"

int X_value, Y_value, Z_value;
double module, angle;

void HMC5883L_init(uint8_t regA, uint8_t regB)
{
	Wire.beginTransmission(HMC5883L_ADDR);
	Wire.write(0x00);
	Wire.write(regA); // 0x70

	Wire.write(0x01);
	Wire.write(regB); // 0xA0   
	Wire.endTransmission();
}

int HMC5883L_read_axis(uint8_t axis)
{
	int res;
	
	Wire.beginTransmission(HMC5883L_ADDR);
	Wire.write(0x02);
	Wire.write(0x01);
	Wire.endTransmission();
	
	delay(6);

	Wire.beginTransmission(HMC5883L_ADDR);
	Wire.write(axis);
	Wire.endTransmission();

	Wire.requestFrom(HMC5883L_ADDR, 2);
	res = Wire.read() << 8;
	res |= Wire.read();

	return res;
}

void HMC5883L_update(void)
{
	X_value = HMC5883L_read_axis(HMC5883L_X);  
	Y_value = HMC5883L_read_axis(HMC5883L_Y);
	Z_value = HMC5883L_read_axis(HMC5883L_Z);

	module = X_value * X_value + Y_value * Y_value + Z_value * Z_value;
	module = sqrt(module);

	// Compas angle in XY-plan
	angle = atan2(Y_value, X_value) * (180 / 3.14159265);
	if (angle < 0) 	       
		angle = angle + 360; 
}
