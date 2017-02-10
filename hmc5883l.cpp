/*
 * Digital compas HMC5883L
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "hmc5883l.h"

#define DEG2RAD 0.017453278f
#define RAD2DEG 57.2957786f

int X_axis, Y_axis, Z_axis;

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

void HMC5883L_get_heading_angle(float angle_x, float angle_y, float *angle_z)
{
	/*
	 * MPU6050 and HMC5883L referentials are not aligned, we need first 
	 * to project the magnetic vector in the MPU referential to match angles.
	 */
	
	int xp = -Y_axis, yp = X_axis, zp = Z_axis;
	float x_rad = angle_x * DEG2RAD, y_rad = angle_y * DEG2RAD;
	
	// https://www.pololu.com/file/0J434/LSM303DLH-compass-app-note.pdf
	
	// equation 2, page 7
	double xh = xp * cos(y_rad) + zp * sin(y_rad),
		yh = xp * sin(x_rad) * sin(y_rad) + yp * cos(x_rad) - Z_axis * sin(x_rad) * cos(y_rad);
	
	// equation 13, page 23
	float psi = 0;
	
	if (abs(xh) < 0.00001) { // avoid floating point comparison to 0.0f
		if (yh < 0)
			psi = M_PI / 2; // = 90 deg
		else if (yh > 0)
			psi = 3 * M_PI / 2; // = 270 deg
	} else {
		psi = atan2(yh, xh);

		if (xh < 0)
			psi += M_PI; // + 180 deg
		else {
			if (yh <= 0)
				psi += 2 * M_PI; // + 360 deg
		}
	}

	*angle_z = psi * RAD2DEG;
}
