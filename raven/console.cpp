#include "raven.h"
#include "console.h"
#include "buzzer.h"

// Restart the Teensy (disconnect USB) (To be tested)
#define RESTART_ADDR		0xE000ED0C
#define WRITE_RESTART(val)	((*(volatile uint32_t *)RESTART_ADDR) = val)
#define TEENSY_REBOOT		WRITE_RESTART(0x5FA0004) 

// The serial port to use
HardwareSerial *serial;
// Internal state (declared static to not overwrite state in raven.ino)
static state_t *state;

// Line to process
String line = "";
// Read a newline character
bool line_complete = false;

// Delay (ms) for status update
uint16_t delay_ms = 500;

// Settings
#define FLOAT_TYPE 	0
#define INTEGER_TYPE 	1

typedef struct { 
	const char *name;
	const char *help;
	uint8_t type;
} setting_t;

setting_t settings[] = {
	{         "alpha", "Complementary filter coefficient", 	FLOAT_TYPE },
	{         "acc_x", "Accelerometer X offset", 		INTEGER_TYPE }, 
	{         "acc_y", "Accelerometer Y offset", 		INTEGER_TYPE }, 
	{         "acc_z", "Accelerometer Z offset", 		INTEGER_TYPE }, 
	{        "gyro_x", "Gyroscope X offset", 		INTEGER_TYPE }, 
	{        "gyro_y", "Gyroscope Y offset", 		INTEGER_TYPE }, 
	{        "gyro_z", "Gyroscope Z offset", 		INTEGER_TYPE }, 
	{          "decl", "Compass declinaison (deg)", 	FLOAT_TYPE },
	{      "yaw_rate", "Yaw max. rate (deg.s-1)", 		FLOAT_TYPE },
	{    "pitch_rate", "Pitch max. rate (deg.s-1)", 	FLOAT_TYPE },
	{     "roll_rate", "Roll max. rate (deg.s-1)", 		FLOAT_TYPE },
	{   "pitch_angle", "Pitch max. angle (deg)", 		FLOAT_TYPE },
	{    "roll_angle", "Roll max. angle (deg)", 		FLOAT_TYPE },
	{    "pid_sample", "PID sample time (ms)", 		FLOAT_TYPE },
	{   "pid_yaw_min", "PID Yaw min value", 		FLOAT_TYPE },
	{   "pid_yaw_max", "PID Yaw max value", 		FLOAT_TYPE },
	{    "pid_yaw_kp", "PID Yaw Kp value", 			FLOAT_TYPE },
	{    "pid_yaw_ki", "PID Yaw Ki value", 			FLOAT_TYPE },
	{    "pid_yaw_kd", "PID Yaw Kd value", 			FLOAT_TYPE },
	{ "pid_pitch_min", "PID Pitch min value", 		FLOAT_TYPE },
	{ "pid_pitch_max", "PID Pitch max value", 		FLOAT_TYPE },
	{  "pid_pitch_kp", "PID Pitch Kp value", 		FLOAT_TYPE },
	{  "pid_pitch_ki", "PID Pitch Ki value", 		FLOAT_TYPE },
	{  "pid_pitch_kd", "PID Pitch Kd value", 		FLOAT_TYPE },
	{  "pid_roll_min", "PID Roll min value", 		FLOAT_TYPE },
	{  "pid_roll_max", "PID Roll max value", 		FLOAT_TYPE },
	{   "pid_roll_kp", "PID Roll Kp value", 		FLOAT_TYPE },
	{   "pid_roll_ki", "PID Roll Ki value", 		FLOAT_TYPE },
	{   "pid_roll_kd", "PID Roll Kd value", 		FLOAT_TYPE },
	{ 		0, 0,					0 }
};	

void print_help()
{
	serial->println(F("List of supported commands:"));
	serial->println(F("   show - print the in-memory configuration"));
	serial->println(F("   load - load the configuration from EEPROM"));
	serial->println(F("   save - save configuration to EEPROM"));
	serial->println(F("  reset - reset the configuration to factory default both in EEPROM & in memory"));
	serial->println(F("    set - set the value of a settings"));
	serial->println(F("   list - list the names of available settings"));
	serial->println(F(" status - display some information about the status of the drone"));
	serial->println(F(" reboot - reboot the Teensy board"));
}

bool yes_or_no()
{
	while(!serial->available()) ;

	char c = serial->read();
	serial->println();
	
	return (c == 'y');
}

void set(String &var, String &val)
{
	float fval = 0;
	int16_t ival = 0;
	char *endptr;
	
	setting_t *p = 0;
	
	for(p = settings; p->name != 0; p ++) {
		if (strcmp(p->name, var.c_str()) == 0) 
			break;
	}

	if (p->name == 0) {
		serial->print(F("unknown settings '"));
		serial->print(var);
		serial->println(F("'"));

		return;
	}

	if (p->type == INTEGER_TYPE) {
		ival = strtol(val.c_str(), &endptr, 10);
		
		if (*endptr != '\0') {
			serial->print(F("invalid value '"));
			serial->print(val);
			serial->println(F("'"));

			return;
		}
	} else if (p->type == FLOAT_TYPE) {
		fval = strtof(val.c_str(), &endptr);
		
		if (*endptr != '\0') {
			serial->print(F("invalid value '"));
			serial->print(val);
			serial->println(F("'"));

			return;
		}
	}

	if (var.equals("alpha"))
		state->config.alpha = fval;
	else if (var.equals("acc_x")) 
		state->config.off_ax = ival;
	else if (var.equals("acc_y"))
		state->config.off_ay = ival;
	else if (var.equals("acc_z")) 
		state->config.off_az = ival;
	else if (var.equals("gyro_x"))
		state->config.off_gx = ival;
	else if (var.equals("gyro_y")) 
		state->config.off_gy = ival;
	else if (var.equals("gyro_z"))
		state->config.off_gz = ival;
	else if (var.equals("decl")) 
		state->config.off_decl = fval;
	else if (var.equals("yaw_rate"))
		state->config.yaw_max_rate = fval;
	else if (var.equals("pitch_rate"))
		state->config.pitch_max_rate = fval;
	else if (var.equals("roll_rate"))
		state->config.roll_max_rate = fval;
	else if (var.equals("pitch_angle"))
		state->config.pitch_max_angle = fval;
	else if (var.equals("roll_angle"))
		state->config.roll_max_angle = fval;
	else if (var.equals("pid_sample"))
		state->config.pid_sample = fval;
	else if (var.equals("pid_yaw_min"))
		state->config.yaw_pid_min = fval;
	else if (var.equals("pid_yaw_max"))
		state->config.yaw_pid_max = fval;
	else if (var.equals("pid_yaw_kp"))
		state->config.yaw_kp = fval;
	else if (var.equals("pid_yaw_ki"))
		state->config.yaw_ki = fval;
	else if (var.equals("pid_yaw_kd"))
		state->config.yaw_kd = fval;
	else if (var.equals("pid_pitch_min"))
		state->config.pitch_pid_min = fval;
	else if (var.equals("pid_pitch_max"))
		state->config.pitch_pid_max = fval;
	else if (var.equals("pid_pitch_kp"))
		state->config.pitch_kp = fval;
	else if (var.equals("pid_pitch_ki"))
		state->config.pitch_ki = fval;
	else if (var.equals("pid_pitch_kd"))
		state->config.pitch_kd = fval;
	else if (var.equals("pid_roll_min"))
		state->config.roll_pid_min = fval;
	else if (var.equals("pid_roll_max"))
		state->config.roll_pid_max = fval;
	else if (var.equals("pid_roll_kp"))
		state->config.roll_kp = fval;
	else if (var.equals("pid_roll_ki"))
		state->config.roll_ki = fval;
	else if (var.equals("pid_roll_kd"))
		state->config.roll_kd = fval;
}

void process_line()
{	
	if (line.length() == 0)
		return;

	if (line.equals("help"))
		print_help();
	else if (line.equals("show"))
		eeprom_dump(&state->config, serial);
	else if (line.equals("load")) {
		serial->print(F("all modifications not saved will be lost, are you sure (y/N)? "));
		if (yes_or_no()) {
			if (eeprom_load(&state->config))
				serial->println("done");
			else 
				serial->println("invalid CRC");
		} 
	} else if (line.equals("save")) {
		serial->print(F("EEPROM settings will be overwritten by current settings, are you sure (y/N)? "));
		if (yes_or_no()) {
			eeprom_save(&state->config);
			serial->println("done");
		}
	} else if (line.equals("reset")) {
		serial->print(F("all modifications not saved will be lost, are you sure (y/N)? "));
		if (yes_or_no()) {
			eeprom_reset(&state->config);
			serial->println("done");
		}
	} else if (line.equals("reboot")) {
		serial->print(F("all modifications not saved will be lost, are you sure (y/N)? "));
		if (yes_or_no()) {
			buzzer_tone(440, 100);
			delay(10);
			buzzer_tone(440, 100);
			delay(10);
			buzzer_tone(440, 100);
			delay(10);			

			TEENSY_REBOOT;		
		}
	} else if (line.equals("list")) {
		serial->println(F("List of settings to use with 'set':"));
		for(setting_t *p = settings; p->name != 0; p ++) {
			serial->print("  ");
			serial->print(p->name);
			serial->print(": ");
			serial->println(p->help);	
		}
	} else if (line.startsWith("set")) {
		int i_var = line.indexOf(" ");
		if (i_var == -1) {
			serial->println(F("no settings name specified"));
			return;
		}

		int i_val = line.lastIndexOf(" ");
		if (i_val <= i_var) {
			serial->println(F("no value specified"));
			return;
		}

		String var = line.substring(i_var+1, i_val), val = line.substring(i_val+1);

		set(var, val);
	} else if (line.startsWith("status")) {
#define LINE_SZ 80
		char stline[LINE_SZ];
		
		unsigned long now = millis();
		
		do {
			get_raw_value(state);
			update_attitude(state, 10000);

			if (millis() - now > delay_ms) {

				// Add some spaces at the end of the line to clear the previous characters
				int n = snprintf(stline, LINE_SZ, " yaw (r/h): %.1f/%.1f pitch (a/r): %.1f/%.1f roll (a/r): %.1f/%.1f          ", 
						state->attitude.yaw_rate, state->attitude.heading,
						state->attitude.pitch, state->attitude.pitch_rate, 
						state->attitude.roll, state->attitude.roll_rate);
						
				serial->print(stline);
				
				for(int i = 0; i < n; i ++) 
					serial->print('\b');	

				now = millis();
			}

			if (serial->available()) {
				serial->read();
				break;
			}

			delay(10);
		} while(true);

		serial->println();
		serial->println("interrupted");

	} else {
		serial->print(F("unknown command '"));
		serial->print(line);
		serial->println(F("'"));
	}
}

void start_console(HardwareSerial &_serial, state_t *_state)
{
	char c;

	serial = &_serial;
	state = _state;
	
	// Say hello
	serial->println(F("\r\n-----------------"));
	serial->print(F("raven v"));
	serial->print(RAVEN_VERSION);
	serial->println(F(", hi!"));

	serial->print("raven> ");

	while(true) {
		if (line_complete) {
			line.trim();
			process_line();

			// Reset
			line = "";
			line_complete = false;

			serial->print("raven> ");
		} else {
			if (serial->available()) {
				c = serial->read();
	
				if (c == '\r' || c == '\n') {
					line_complete = true;
				} else 
					line += c;
			}
		}
	}
}

