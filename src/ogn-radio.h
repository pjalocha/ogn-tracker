#pragma once

#include <stdint.h>

#include "main.h"

#include "format.h"
#include "ogn.h"
#include "ognconv.h"
#include "ogn1.h"
#include "ldpc.h"
#include "fifo.h"
#include "freqplan.h"
#include "rx-pkt.h"

#include "paw.h"

const uint8_t Radio_SysID_FLR  = 0;  //
const uint8_t Radio_SysID_OGN  = 1;
const uint8_t Radio_SysID_ADSL = 2;
const uint8_t Radio_SysID_RID  = 3;
const uint8_t Radio_SysID_FNT  = 4;  // LoRa modulation
const uint8_t Radio_SysID_PAW  = 5;  // PAW or ADS-L LDR

extern const char *Radio_SysName[6];

extern FIFO<OGN_TxPacket<OGN_Packet>, 4> OGN_TxFIFO;
extern FIFO<ADSL_Packet,              4> ADSL_TxFIFO;
// extern FIFO<ADSL_RID,                 4> RID_TxFIFO;
extern FIFO<FANET_Packet,             4> FNT_TxFIFO;
extern FIFO<PAW_Packet,               4> PAW_TxFIFO;

extern FIFO<FSK_RxPacket,            64> FSK_RxFIFO;
extern FIFO<FANET_RxPacket,           8> FNT_RxFIFO;

extern FreqPlan Radio_FreqPlan;       // RF frequency hopping scheme
extern QueueHandle_t Radio_SlotMsg;   // to tell the Radio_Task about the new time-slot

extern uint32_t Radio_TxCount[6];     // transmitted packet counters
extern uint32_t Radio_RxCount[6];     // received packet counters
extern  int32_t Radio_TxCredit;
extern float    Radio_BkgRSSI;        // [dBm] background noise seen by the receiver
extern float    Radio_PktRate;        // [Hz]

void Radio_Task(void *Parms);

#ifdef WITH_LORAWAN
#include "lorawan.h"
extern LoRaWANnode WANdev;
#endif
