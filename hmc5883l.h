#ifndef _RAVEN_HMC5883L_H
#define _RAVEN_HMC5883L_H

// I2C module address
#define HMC5883L_ADDR 0x1E  

// Register addresses for (XYZ)-axis
#define HMC5883L_X 3  
#define HMC5883L_Y 7
#define HMC5883L_Z 5

/*
 * Initialization
 * 
 * regA: configure the device for setting the data output rate and measurement configuration
 * regB: set the device gain
 * 
 * See HMC5883L specs for detail.  
 */
void HMC5883L_init(uint8_t regA, uint8_t regB);

/*
 * Update all data from this sensor
 */
void HMC5883L_update(void);

/*
 * Returns the tilt compensated heading angle (in radians)
 */
void HMC5883L_get_heading_angle(float pitch, float roll, float *heading);

#endif // _RAVEN_HMC5883L_H
