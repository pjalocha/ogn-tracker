#pragma once

#ifdef WITH_ODID_RX

#include "esp_err.h"
#include <stdint.h>

typedef struct
{ uint32_t All;
  uint32_t Beacon;
  uint32_t Nan;
  uint32_t FrenchID;
} ODID_RX_Counters;

esp_err_t ODID_RX_Init(void);
// Call these from a task context after ODID_RX_Init(). Stop shuts the WiFi driver
// down; Start restarts it on the configured sniffer channel.
esp_err_t ODID_RX_Start(void);
esp_err_t ODID_RX_Stop(void);
void ODID_RX_GetCounters(ODID_RX_Counters *Counters);
// Drain recognized Remote ID and FrenchID frames from the sniffer queue in the
// PROC task. The queued data is retained for later protocol decoding there.
void ODID_RX_ProcessPending(void);

#endif // WITH_ODID_RX
