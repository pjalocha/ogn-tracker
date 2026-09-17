#pragma once

#if defined(WITH_MOBILE) && defined(WITH_WIFI)

#include "nmea.h"
#include "adsl.h"
#include "ogn1.h"

void MOBILE_Init(void);
bool MOBILE_SendADSL(const ADSL_Packet &Packet);
bool MOBILE_SendOGN(const OGN1_Packet &Packet);
bool MOBILE_Flush(void);

#endif
