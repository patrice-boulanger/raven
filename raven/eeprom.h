#ifndef _RAVEN_EEPROM_H
#define _RAVEN_EEPROM_H

#include <Arduino.h>
#include <EEPROM.h>

typedef struct {
	// Checksum
	unsigned long chksum;
	// Complementary filter coefficient
	float alpha;
	// MPU offsets
	int off_ax, off_ay, off_az;
	int off_gx, off_gy, off_gz;
	// Compass declinaison
	float off_decl; 
	// Acro mode settings
	float yaw_max_rate, pitch_max_rate, roll_max_rate;
	// Auto leveled mode settings
	float pitch_max_angle, roll_max_angle;
	// PID sample time in ms
	unsigned int pid_sample;
	// PID yaw
	float yaw_pid_min, yaw_pid_max;
	float yaw_kp, yaw_ki, yaw_kd;
	// PID pitch
	float pitch_pid_min, pitch_pid_max;
	float pitch_kp, pitch_ki, pitch_kd;
	// PID roll
	float roll_pid_min, roll_pid_max;
	float roll_kp, roll_ki, roll_kd;
} eeprom_config_t;

/*
 * Load & store a configuration from EEPROM
 * Returns true if the checksum is valid, false otherwise
 */
bool eeprom_load(eeprom_config_t *conf);

/*
 * Save a configuration to EEPROM w/ CRC
 */
void eeprom_save(eeprom_config_t *conf);

/*
 * Reset the configuration w/ default configuration both in EEPROM & in memory
 */
void eeprom_reset(eeprom_config_t *conf);

/*
 * Dump the configuration on Serial
 */
void eeprom_dump(const eeprom_config_t *conf, HardwareSerial *serial);

#endif // _RAVEN_EEPROM_H
