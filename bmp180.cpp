#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "bmp180.h"

int ac1, ac2, ac3, vb1, vb2, mb, mc, md;
unsigned int ac4, ac5, ac6; 
float c5, c6, x0, x1, x2, y0, y1, y2, p0, p1, p2;

// oversampling settings
uint8_t osd, cmd_pressure;

// temperature in celsius degrees
float deg_celsius;
// relative pressure in mbar
float pressure_mbar;
// absolute & relative altitude
float abs_alt, rel_alt;

uint16_t read_2(uint8_t code)
{
	uint16_t value = 0;

	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(code);
	Wire.endTransmission();

	Wire.requestFrom(BMP180_ADDR, 2);
	while(Wire.available() < 2)
		;
 
	value = (Wire.read() << 8) | Wire.read();
  
	return value;
}

void read_bytes(uint8_t *val, int len)
{
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(val[0]);
	Wire.endTransmission();
	
	Wire.requestFrom(BMP180_ADDR, len);
	while(Wire.available() < len)
		;
	
	for(int x = 0; x < len; x ++)
		val[x] = Wire.read();
}

void write_bytes(uint8_t *val, int len)
{
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(val, len);
	Wire.endTransmission();
}

void BMP180_init(uint8_t resolution)
{
	float c3, c4, b1;
  
	switch(resolution) {
	case BMP180_RES_ULTRA_LOW:
		cmd_pressure = BMP180_COMMAND_PRESSURE0;
		osd = 5;
		break;

	case BMP180_RES_STANDARD:
		cmd_pressure = BMP180_COMMAND_PRESSURE1;
		osd = 8;
		break;

	case BMP180_RES_HIGH:
		cmd_pressure = BMP180_COMMAND_PRESSURE2;
		osd = 14;
		break;

	case BMP180_RES_ULTRA_HIGH:
	default:
		cmd_pressure = BMP180_COMMAND_PRESSURE3;
		osd = 26;
		break;
	}

	// get calibration data from device
	ac1 = read_2(0xAA);
	ac2 = read_2(0xAC);
	ac3 = read_2(0xAE);
	ac4 = read_2(0xB0);
	ac5 = read_2(0xB2);
	ac6 = read_2(0xB4);
	vb1 = read_2(0xB6);
	vb2 = read_2(0xB8);
	mb  = read_2(0xBA);
	mc  = read_2(0xBC);
	md  = read_2(0xBE);

	// get floating-point polynominals
	c3 = 160.0 * pow(2,-15) * ac3;
	c4 = pow(10,-3) * pow(2,-15) * ac4;
	b1 = pow(160,2) * pow(2,-30) * vb1;
	c5 = (pow(2,-15) / 160) * ac5;
	c6 = ac6;
	mc = (pow(2,11) / pow(160,2)) * mc;
	md = md / 160.0;
	x0 = ac1;
	x1 = 160.0 * pow(2,-13) * ac2;
	x2 = pow(160,2) * pow(2,-25) * vb2;
	y0 = c4 * pow(2,15);
	y1 = c4 * c3;
	y2 = c4 * b1;
	p0 = (3791.0 - 8.0) / 1600.0;
	p1 = 1.0 - 7357.0 * pow(2,-20);
	p2 = 3038.0 * 100.0 * pow(2,-36);

	/*
	  Serial.println();
	  Serial.print("c3: "); Serial.println(c3);
	  Serial.print("c4: "); Serial.println(c4);
	  Serial.print("c5: "); Serial.println(c5);
	  Serial.print("c6: "); Serial.println(c6);
	  Serial.print("b1: "); Serial.println(b1);
	  Serial.print("mc: "); Serial.println(mc);
	  Serial.print("md: "); Serial.println(md);
	  Serial.print("x0: "); Serial.println(x0);
	  Serial.print("x1: "); Serial.println(x1);
	  Serial.print("x2: "); Serial.println(x2);
	  Serial.print("y0: "); Serial.println(y0);
	  Serial.print("y1: "); Serial.println(y1);
	  Serial.print("y2: "); Serial.println(y2);
	  Serial.print("p0: "); Serial.println(p0);
	  Serial.print("p1: "); Serial.println(p1);
	  Serial.print("p2: "); Serial.println(p2);
	*/
}

void BMP180_update(void)
{
	uint8_t data[3];
	
	// Get temperature
	data[0] = BMP180_REG_CONTROL;
	data[1] = BMP180_COMMAND_TEMPERATURE;
	write_bytes(data, 2);
	
	delay(osd);
	
	data[0] = BMP180_REG_RESULT;
	read_bytes(data, 2);
	
	float tu = (data[0] * 256.0) + data[1], a = c5 * (tu - c6);
	deg_celsius = a + (mc / (a + md));

	/*
	  Serial.println();
	  Serial.print("tu: "); Serial.println(tu);
	  Serial.print("a: "); Serial.println(a);
	*/
	
	// Get pressure 
	data[0] = BMP180_REG_CONTROL;
	data[1] = cmd_pressure;
	write_bytes(data, 2);
	
	delay(osd);
	
	data[0] = BMP180_REG_RESULT;
	read_bytes(data, 3);
	
	float pu = (data[0] * 256.0) + data[1] + (data[2]/256.0),
		s = deg_celsius - 25.0,
		x = (x2 * pow(s, 2)) + (x1 * s) + x0,
		y = (y2 * pow(s, 2)) + (y1 * s) + y0,
		z = (pu - x) / y;
	
	pressure_mbar = (p2 * pow(z, 2)) + (p1 * z) + p0;

	/*
	  Serial.println();
	  Serial.print("pu: "); Serial.println(pu);
	  Serial.print("s: "); Serial.println(s);
	  Serial.print("x: "); Serial.println(x);
	  Serial.print("y: "); Serial.println(y);
	  Serial.print("z: "); Serial.println(z);
	*/
  
	abs_alt = 44330.0 * (1.0 - pow(pressure_mbar / 1013.25, 1.0 / 5.255));
/*  
    Serial.print("BMP180: ");
    Serial.print(deg_celsius);
    Serial.print(" deg. ");
    Serial.print(pressure_mbar);
    Serial.print(" mbar ");
    Serial.print(abs_alt);
    Serial.println(" m");
*/
}
