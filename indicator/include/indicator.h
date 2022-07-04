#ifndef INDICATOR_H
#define INDICATOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief		Indicator tick
 */
void indicator_tick(void);


void indicator_buzzer_beep(uint16_t length);

void indicator_buzzer_beeps(uint8_t count);

bool indicator_is_beeping(void);

#endif /* INDICATOR_H */
