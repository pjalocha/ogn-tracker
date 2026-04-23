#pragma once

const uint32_t Flasher_PattDouble = 0x01100000;
const uint32_t Flasher_PattTriple = 0x01110000;

void Flasher_Init(void);
void Flasher_ON(bool ON=1);

void Flasher_Play(uint32_t Patt);
void Flasher_TimerCheck(uint8_t Ticks);
