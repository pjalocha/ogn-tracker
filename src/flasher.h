#pragma once

void Flasher_Init(void);
void Flasher_ON(bool ON=1);

void Flasher_Play(uint32_t Patt);
void Flasher_TimerCheck(uint8_t Ticks);
