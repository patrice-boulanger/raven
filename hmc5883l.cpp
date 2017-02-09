/*
 * Digital compas HMC5883L
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "hmc5883l.h"

int X_axis, Y_axis, Z_axis;
double angle;

void HMC5883L_init(uint8_t regA, uint8_t regB)
{
	Wire.beginTransmission(HMC5883L_ADDR);
	Wire.write(0x00);
	Wire.write(regA);

	Wire.write(0x01);
	Wire.write(regB);   

	Wire.write(0x02); //select mode register
	Wire.write(0x00); //continuous measurement mode
	Wire.endTransmission();
}

void HMC5883L_update(void)
{
	Wire.beginTransmission(HMC5883L_ADDR);
	Wire.write(0x03); //select register 3, X MSB register
	Wire.endTransmission();
  
	//Read data from each axis, 2 registers per axis
	Wire.requestFrom(HMC5883L_ADDR, 6);
	
	while(Wire.available() < 6)
		;
	
	X_axis = Wire.read() << 8; // X msb
	X_axis |= Wire.read();     // X lsb
	Z_axis = Wire.read() << 8; // Z msb
	Z_axis |= Wire.read();     // Z lsb
	Y_axis = Wire.read() << 8; // Y msb
	Y_axis |= Wire.read();     // Y lsb
}

void HMC5883L_get_heading_angle(float pitch, float roll, float *heading)
{
	// https://www.pololu.com/file/0J434/LSM303DLH-compass-app-note.pdf

	// equation 2, page 7
	double xh = X_axis * cos(pitch) + Z_axis * sin(pitch),
		yh = X_axis * sin(roll) * sin(pitch) + Y_axis * cos(roll) - Z_axis * sin(roll) * cos(pitch);


	// equation 13, page 23
	float psi = atan2(yh, xh);

	if (xh < 0)
		psi += M_PI; // + 180 deg
	else if (xh > 0 && yh <= 0) 
		psi += 2 * M_PI; // + 360 deg
	else if (xh == 0) {
		if (yh < 0)
			psi = M_PI / 2; // = 90 deg
		else if (yh > 0)
			psi = 3 * M_PI / 2; // = 270 deg
	}

	*heading = psi;
}
