#pragma once

#ifdef WITH_ODID_RX

#include "esp_err.h"
#include <stdint.h>
#include "ogn.h"

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
// Diagnostic-only drain/print. This consumes queued frames; do not call it
// alongside either ODID_RX_GetPacket() overload.
void ODID_RX_ProcessPending(void);

// Consume exactly one queued frame. Returns -1 when the queue is empty, 0 when
// it has no usable position, and 1 when a Beacon/NAN/FrenchID position was
// converted. GeoidSepar is in meters; FrenchID altitude is already AMSL.
int ODID_RX_GetPacket(OGN_RxPacket<OGN1_Packet> &Packet, int16_t GeoidSepar);
int ODID_RX_GetPacket(ADSL_RxPacket &Packet, int16_t GeoidSepar);

#endif // WITH_ODID_RX
