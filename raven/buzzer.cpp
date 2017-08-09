#include "raven.h"
#include "buzzer.h"

/*
 * A melody is a sequence of frequency/duration, ended by 0/0:
 *
 * { freq, duration, freq, duration, ..., 0, 0 }
 */
// BUZZER_START
int buz_start[] = { 440, 500, 440, 250, 440, 250, 0, 0 };
// BUZZER_READY
int buz_ready[] = { 220, 500, 440, 500, 880, 500, 0, 0 };
 
int * melodies[] = {
        buz_start, 
        buz_ready
};

void buzzer_play(uint8_t melody)
{
	if (melody >= sizeof(melodies) / sizeof(melodies[0]))
		return;

	for(int *freq = melodies[melody], *duration = (freq+1) ; *freq != 0; freq ++, duration ++) {
		tone(BUZZER_PIN, *freq, *duration);
		delay(*duration + 10);
	}

        noTone(BUZZER_PIN);
}
	
