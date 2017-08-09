#include "led.h"

#define SEQ_SZ      64

char sequence[SEQ_SZ + 1];
int seq_len = 0, seq_pos = -1;

void led_on(uint8_t pin)
{
	analogWrite(LED_GREEN_PIN, 0);
}

void led_off(uint8_t pin)
{
	analogWrite(LED_GREEN_PIN, 255);
}

void led_clear()
{
	led_off(LED_GREEN_PIN);
	led_off(LED_RED_PIN);
	led_off(LED_WHITE_PIN);
}

void led_sequence(const char *seq)
{
	// Switch off all LEDs
	led_clear();
	
	if (seq == 0) 
		seq_pos = -1;
	else {
		strncpy(sequence, seq, SEQ_SZ);
		seq_len = strlen(sequence);
		seq_pos = 0;
	}
}

void led_update()
{
	if (seq_pos == -1)
		return;
	
	char c = sequence[seq_pos];
	
	// Increase the position, restart from the beginning if needed
	seq_pos = (seq_pos ++) % seq_len;
	
	switch(c) {
	case 'g':
		led_off(LED_GREEN_PIN);
		break;
	case 'G':
		led_on(LED_GREEN_PIN);
		break;
	case 'r':
		led_off(LED_RED_PIN);
		break;
	case 'R':
		led_on(LED_RED_PIN);
		break;
	case 'w':
		led_off(LED_WHITE_PIN);
		break;
	case 'W':
		led_on(LED_WHITE_PIN);
		break;
	}
}
