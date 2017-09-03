#include "raven.h"
#include "buzzer.h"

/*
 * A melody is a sequence of frequency/duration, ended by 0/0/0:
 *
 * { freq, duration, pause, freq, duration, pause, ..., 0, 0, 0 }
 */
// BUZZER_START
int buz_start[] = { 2200, 100, 150, 2200, 100, 150, 2200, 100, 150, 0, 0, 0 };
// BUZZER_READY
int buz_ready[] = { 2200, 200, 250, 2200, 50, 60, 2200, 50, 60, 0, 0, 0 };
 
int * melodies[] = {
        buz_start, 
        buz_ready
};

void buzzer_tone(uint16_t pulsation, uint16_t duration)
{
	tone(BUZZER_PIN, pulsation, duration);
	delay(duration * 1.1);
        noTone(BUZZER_PIN);
}

void buzzer_play(uint8_t melody)
{
	if (melody >= sizeof(melodies) / sizeof(melodies[0]))
		return;

	for(int *freq = melodies[melody], *duration = (freq+1), *pause = (freq+2); 
		*freq != 0 && *duration != 0 && *pause != 0; 
			freq += 2, duration += 2, pause += 2) {
				
		tone(BUZZER_PIN, *freq, *duration);
		delay(*pause);
	}

        noTone(BUZZER_PIN);
}
	
