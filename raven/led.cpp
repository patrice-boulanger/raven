#include "led.h"

#define SEQ_SZ      64

char sequence[SEQ_SZ + 1];
int seq_len = 0, seq_pos = -1;

void set_leds(const char *seq)
{
	// Switch off all LEDs
	analogWrite(LED_GREEN_PIN, 0);
	analogWrite(LED_RED_PIN, 0);
	analogWrite(LED_WHITE_PIN, 0);
	
	if (seq == 0) 
		seq_pos = -1;
	else {
		strncpy(sequence, seq, SEQ_SZ);
		seq_len = strlen(sequence);
		seq_pos = 0;
	}
}

void update_leds()
{
	if (seq_pos == -1)
		return;
	
	char c = sequence[seq_pos];
	
	// Increase the position, restart from the beginning if needed
	seq_pos = (seq_pos ++) % seq_len;
	
	switch(c) {
	case 'g':
		analogWrite(LED_GREEN_PIN, 0);
		break;
	case 'G':
		analogWrite(LED_GREEN_PIN, 255);
		break;
	case 'r':
		analogWrite(LED_RED_PIN, 0);
		break;
	case 'R':
		analogWrite(LED_RED_PIN, 255);
		break;
	case 'w':
		analogWrite(LED_WHITE_PIN, 0);
		break;
	case 'W':
		analogWrite(LED_WHITE_PIN, 255);
		break;
	}
}
