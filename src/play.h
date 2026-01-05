#pragma once

const uint8_t Play_Vol_0 = 0x00;
const uint8_t Play_Vol_1 = 0x40;
const uint8_t Play_Vol_2 = 0x80;
const uint8_t Play_Vol_3 = 0xC0;

const uint8_t Play_Oct_0 = 0x00;
const uint8_t Play_Oct_1 = 0x10;
const uint8_t Play_Oct_2 = 0x20;
const uint8_t Play_Oct_3 = 0x30;

void Beep_Init(void);

void Beep(uint16_t Freq, uint8_t Duty=127, uint8_t DoubleAmpl=0);

void Beep_Note(uint8_t Note);

void Play(uint8_t Note, uint8_t Len);

uint8_t Play_isBusy(void);

void Play_TimerCheck(uint8_t Ticks=1);

