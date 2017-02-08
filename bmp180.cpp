#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "bmp180.h"

int16_t  ac1, ac2, ac3, b1, b2, mb, mc, md; // Store sensor PROM values from BMP180
uint16_t ac4, ac5, ac6;

// oversampling setting & oversampling delay
uint8_t oss, osd;

// temperature in celsius degrees
float deg_celsius;
// initial & relative pressure in mbar
float P0, pressure;
// absolute & relative altitude
float abs_alt, rel_alt;

uint16_t read_2(uint8_t code)
{
	uint16_t value = 0;

	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(code);
	Wire.endTransmission();

	Wire.requestFrom(BMP180_ADDR, 2);
	if(Wire.available() >= 2) {
		value = (Wire.read() << 8) | Wire.read();
	}

	return value;
}

int32_t read_pressure()
{
	int32_t P;

	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(0xf4);
	Wire.write(0x34 + (oss << 6));
	Wire.endTransmission();
	
	delay(osd);

	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(0xf6);
	Wire.endTransmission();
	
	Wire.requestFrom(BMP180_ADDR, 3);
	if(Wire.available() >= 3)
		P = (((int32_t)Wire.read() << 16) | ((int32_t)Wire.read() << 8) | ((int32_t)Wire.read())) >> (8 - oss);

	return P;                             
}

/*
 * Returns pressure in mbar 
 */
float get_pressure(int32_t b5)
{
	int32_t x1, x2, x3, b3, b6, p, UP;
	uint32_t b4, b7;

	UP = read_pressure();

	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = (((ac1 * 4 + x3) << oss) + 2) >> 2;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
	b7 = ((uint32_t)UP - b3) * (50000 >> oss);
	p = b7 < 0x80000000 ? ((b7 << 1) / b4) : ((b7 / b4) << 1);
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	
	return (p + ((x1 + x2 + 3791) >> 4)) / 100.0f; 
}

int32_t get_temperature()
{
	int32_t x1, x2, b5, UT;

	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(0xf4);
	Wire.write(0x2e);
	Wire.endTransmission();
	
	delay(5);

	UT = read_2(0xf6);

	// Real temperature
	x1 = (UT - (int32_t)ac6) * (int32_t)ac5 >> 15;
	x2 = ((int32_t)mc << 11) / (x1 + (int32_t)md);
	b5 = x1 + x2;

	// Returns not-compensated temperature
	return b5;
}

float get_altitude(float p, float ref)
{
	return 44330.0 * (1 - pow(p/ref, 1/5.255));
}

void BMP180_set_baseline(void)
{
	int32_t T = get_temperature();
	P0 = get_pressure(T);
}

void BMP180_init(uint8_t resolution)
{
	switch(resolution) {
	case BMP180_RES_ULTRA_LOW:
		oss = 0;
		osd = 5;
		break;

	case BMP180_RES_STANDARD:
		oss = 1;
		osd = 8;
		break;

	case BMP180_RES_HIGH:
		oss = 2;
		osd = 14;
		break;

	case BMP180_RES_ULTRA_HIGH:
	default:
		oss = 3;
		osd = 26;
		break;
	}
		
	ac1 = read_2(0xAA);
	ac2 = read_2(0xAC);
	ac3 = read_2(0xAE);
	ac4 = read_2(0xB0);
	ac5 = read_2(0xB2);
	ac6 = read_2(0xB4);
	b1  = read_2(0xB6);
	b2  = read_2(0xB8);
	mb  = read_2(0xBA);
	mc  = read_2(0xBC);
	md  = read_2(0xBE);

	BMP180_set_baseline();
}

void BMP180_update(void)
{
	int32_t T = get_temperature();
	// Convert to Celsius degrees
	deg_celsius  = (T + 8) >> 4;
	deg_celsius = deg_celsius / 10.0;
	
	pressure = get_pressure(T);
	// Comput altitudes
	abs_alt = get_altitude(pressure, 1013.25);
	if (P0 != 0) 
		rel_alt = get_altitude(pressure, P0);
	else
		rel_alt = abs_alt;
}
