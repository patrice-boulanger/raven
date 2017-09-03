#include <Arduino.h>
#include "eeprom.h"

/*
 * Default configuration to be used if the EEPROM is not valid
 * or to reset the stored configuration.
 */
const eeprom_config_t default_config = {
	// Checksum
	0L,
	// Complementary filter coefficient
	0.96f,
	// Calibration offsets: computed w/ I2Cdev IMU_Zero sketch
	-989, -492, 1557,  	// accelerometer
	  36,    1,   26,	// gyroscope 
	// Compass declinaison
	0.0f, 
	// Acro mode settings
	5.0f, 5.0f, 5.0f,	// yaw_max_rate, pitch_max_rate, roll_max_rate
	// Auto leveled mode settings
	5.0f, 5.0f,		// pitch_max_angle, roll_max_angle
	// PID sample in ms
	9.0f,			
	// PID yaw
	0.0f, 0.0f,		// min/max values
	0.0f, 0.0f, 0.0f, 	// Kp, Ki, Kd
	// PID pitch
	-100.0f, 100.0f,	// min/max values
	0.9f, 0.02f, 0.1f, 	// Kp, Ki, Kd
	// PID roll
	-100.0f, 100.0f,	// min/max values
	0.9f, 0.02f, 0.1f, 	// Kp, Ki, Kd
};

/*
 * Inspired from Arduino official tutorial
 * https://www.arduino.cc/en/Tutorial/EEPROMCrc
 */
unsigned long eeprom_crc(const eeprom_config_t *conf) 
{	
	const unsigned long crc_table[16] = {
		0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
		0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
		0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
		0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
	};

	// Process the configuration struct as a bytes array
	const char *p = (const char *) conf;
	
  	unsigned long crc = ~0L;
	
  	for (uint16_t i = 0; i < sizeof(eeprom_config_t); i ++) {
    		crc = crc_table[(crc ^ p[i]) & 0x0f] ^ (crc >> 4);
    		crc = crc_table[(crc ^ (p[i] >> 4)) & 0x0f] ^ (crc >> 4);
    		crc = ~crc;
  	}

  	return crc;
}

/*
 * Dump a configuration on Serial
 */
void eeprom_dump(const eeprom_config_t *conf, HardwareSerial *serial)
{
	serial->print("Checksum: 0x");
	serial->println(conf->chksum, HEX);
	
	serial->print("Complementary filter coefficient: ");
	serial->println(conf->alpha, 3);

	serial->println("MPU offsets:");
	serial->print(" Accelerometer: ");
	serial->print(conf->off_ax);
	serial->print(", ");
	serial->print(conf->off_ay);
	serial->print(", ");	
	serial->println(conf->off_az);
	serial->print(" Gyroscope: ");
	serial->print(conf->off_gx);
	serial->print(", ");
	serial->print(conf->off_gy);
	serial->print(", ");	
	serial->println(conf->off_gz);
	
	serial->print("Compass declinaison (deg.): ");
	serial->println(conf->off_decl);

	serial->print("PID Sample time (ms): ");
	serial->println(conf->pid_sample);

	serial->println("Yaw settings:");
	serial->print("  max. rate (deg.s-1): ");
	serial->println(conf->yaw_max_rate);
	serial->print("  PID min/max: ");
	serial->print(conf->yaw_pid_min);
	serial->print("/");
	serial->println(conf->yaw_pid_max);
	serial->print("  Kp: ");
	serial->print(conf->yaw_kp);
	serial->print(" Ki: ");
	serial->print(conf->yaw_ki);
	serial->print(" Kd: ");
	serial->println(conf->yaw_kd);
	 
	serial->println("Pitch settings:");
	serial->print("  max. rate (deg.s-1): ");
	serial->println(conf->pitch_max_rate);
	serial->print("  max. angle (deg): ");
	serial->println(conf->pitch_max_angle);
	serial->print("  PID min/max: ");
	serial->print(conf->pitch_pid_min);
	serial->print("/");
	serial->println(conf->pitch_pid_max);
	serial->print("  Kp: ");
	serial->print(conf->pitch_kp);
	serial->print(" Ki: ");
	serial->print(conf->pitch_ki);
	serial->print(" Kd: ");
	serial->println(conf->pitch_kd);

	serial->println("Roll settings:");
	serial->print("  max. rate (deg.s-1): ");
	serial->println(conf->roll_max_rate);
	serial->print("  max. angle (deg): ");
	serial->println(conf->roll_max_angle);
	serial->print("  PID min/max: ");
	serial->print(conf->roll_pid_min);
	serial->print("/");
	serial->println(conf->roll_pid_max);
	serial->print("  Kp: ");
	serial->print(conf->roll_kp);
	serial->print(" Ki: ");
	serial->print(conf->roll_ki);
	serial->print(" Kd: ");
	serial->println(conf->roll_kd);
}

/*
 * Load & store a configuration from EEPROM
 * Returns true if the checksum is valid, false otherwise
 */
bool eeprom_load(eeprom_config_t *conf)
{	
	//EEPROM.get(0, conf);

	uint8_t *p = (uint8_t *) conf;
	for(uint16_t i = 0; i < sizeof(eeprom_config_t); i ++) {
		*p = EEPROM[i];
		p ++;
	}

	// Backup the checksum & set it to zero
	unsigned long csum = conf->chksum;
	conf->chksum = 0L;

	unsigned long crc = eeprom_crc(conf);
	
	// Restore the checksum
	conf->chksum = csum;

	return (csum == crc);
}

/*
 * Save a configuration to EEPROM w/ CRC
 */
void eeprom_save(eeprom_config_t *conf)
{
	// Set the checksum to zero in the configuration & compute the CRC
	conf->chksum = 0L;
	conf->chksum = eeprom_crc(conf);

	uint8_t *p = (uint8_t *) conf;
	for(uint16_t i = 0; i < sizeof(eeprom_config_t); i ++) {
		EEPROM[i] = *p;
		p ++;
	}
	//EEPROM.put(0, conf);
}

/*
 * Reset the configuration w/ default configuration both in EEPROM & in memory
 */
void eeprom_reset(eeprom_config_t *conf)
{
	memcpy(conf, &default_config, sizeof(eeprom_config_t));
	eeprom_save(conf); // This will compute the checksum
}

