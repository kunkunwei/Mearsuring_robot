#ifndef BUZZER_MUSIC_H
#define BUZZER_MUSIC_H

#include <stdint.h>

void Buzzer_AlertInit(uint32_t now_ms);
void Buzzer_AlertUpdate(uint32_t now_ms, uint8_t motor_offline_mask);

#endif
