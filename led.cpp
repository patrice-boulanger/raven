#include <Arduino.h>

#include "led.h"

#define LED_PIN 13
#define LED_INTRVL 10

const char *led_string = 0;
uint16_t led_sz = 0, pos = 0;
unsigned long tstamp;

void LED_init()
{
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	led_string = 0;
	led_sz = pos = 0;

	tstamp = millis();
}

void LED_set_sequence(const char *str)
{
	led_string = str;
	if (str) 
		led_sz = strlen(str);
	else {
		digitalWrite(LED_BUILTIN, LOW); // force LED off
		led_sz = 0;
	}
	
	pos = 0;
}

void LED_update()
{
	if (!led_string)
		return;
	
	unsigned long now = millis();
	
	if (now - tstamp >= LED_INTRVL) {
		pos = (pos + 1) % led_sz;

		char c = led_string[pos];

		if (c == '_')
			digitalWrite(LED_BUILTIN, LOW);
		else
			digitalWrite(LED_BUILTIN, HIGH);

		tstamp = now;
	}
}
