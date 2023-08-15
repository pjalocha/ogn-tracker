#pragma once

#include <stdint.h>

#include "main.h"

#include "rfm.h"

#include "fifo.h"

#include "freqplan.h"
#include "fanet.h"

extern uint32_t RX_Random;

extern FIFO<RFM_FSK_RxPktData, 16> RF_RxFIFO;   // buffer for received packets
extern FIFO<OGN_TxPacket<OGN_Packet>, 4> RF_TxFIFO;   // buffer for transmitted packets

#ifdef WITH_ADSL
  extern FIFO<ADSL_Packet, 4> ADSL_TxFIFO;
#endif

#ifdef WITH_FANET
  extern FIFO<FANET_RxPacket, 8> FNT_RxFIFO;
  extern FIFO<FANET_Packet, 4> FNT_TxFIFO;
#endif

  extern uint8_t RX_OGN_Packets;              // [packets] counts received packets

  extern FreqPlan  RF_FreqPlan;               // frequency hopping pattern calculator
  extern  int32_t    TX_Credit;               // [ms] counts transmitter time to avoid using more than 1%

  extern uint16_t RX_OGN_Count64;             // counts received packets for the last 64 seconds

  extern RFM_TRX           TRX;               // RF transceiver

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskRF(void* pvParameters);

