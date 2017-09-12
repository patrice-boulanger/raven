#include "raven.h"
#include "buzzer.h"

void buzzer_tone(uint16_t pulsation, uint16_t duration)
{
	tone(BUZZER_PIN, pulsation, duration);
	delay(duration * 1.1);
        noTone(BUZZER_PIN);
}

