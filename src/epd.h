#pragma once

#include "main.h"

#ifdef WITH_EPAPER

#include <SPI.h>
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>

void EPD_Init(void);
void EPD_DrawID(void);
void EPD_UpdateID(void);

void EPD_Task(void *Parms);

#endif
