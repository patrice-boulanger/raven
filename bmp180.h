#ifndef _RAVEN_BMP180_H
#define _RAVEN_BMP180_H

// I2C module address
#define BMP180_ADDR 0x77

#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6

#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0 0x34
#define BMP180_COMMAND_PRESSURE1 0x74
#define BMP180_COMMAND_PRESSURE2 0xB4
#define BMP180_COMMAND_PRESSURE3 0xF4

// Sensor resolution
// Ultra Low Power       OSS = 0, OSD =  5ms
#define BMP180_RES_ULTRA_LOW  0
// Standard              OSS = 1, OSD =  8ms
#define BMP180_RES_STANDARD   1
// High                  OSS = 2, OSD = 14ms
#define BMP180_RES_HIGH       2
// Ultra High Resolution OSS = 3, OSD = 26ms
#define BMP180_RES_ULTRA_HIGH 3

/*
 * Initialization.
 *
 */
void BMP180_init(uint8_t resolution);

/*
 * Update all data from this sensor
 */
void BMP180_update(void);

#endif // _RAVEN_BMP180_H
