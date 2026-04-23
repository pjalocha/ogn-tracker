#pragma once

const uint32_t Flasher_PattDouble = 0b0000011111000001111100000000000000000; // two flashes of 50 ms and 50ms space
const uint32_t Flasher_PattTriple = 0b0000011111000001111100000111110000000; // three flashes like above

void Flasher_Init(void);
void Flasher_ON(bool ON=1);

void Flasher_Play(uint32_t Patt);
void Flasher_TimerCheck(uint8_t Ticks);
