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

	// Compas angle in XY-plan
	angle = atan2(Y_axis, X_axis) * (180 / 3.14159265);
	if (angle < 0) 	       
		angle = angle + 360; 
/*
  Serial.print("HMC5883L: compas angle = ");
  Serial.print(angle);
  Serial.println(" deg");
  */
}
