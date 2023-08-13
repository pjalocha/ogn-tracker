#include <stdint.h>

#include "rf.h"

uint32_t RX_Random = 0x12345678;

FreqPlan  RF_FreqPlan;               // frequency hopping pattern calculator

       FIFO<RFM_FSK_RxPktData, 16> RF_RxFIFO;         // buffer for received packets
       FIFO<OGN_TxPacket<OGN_Packet>, 4> RF_TxFIFO;   // buffer for transmitted packets

#ifdef WITH_ADSL
       FIFO<ADSL_Packet, 4> ADSL_TxFIFO;
#endif

#ifdef WITH_FANET
       FIFO<FANET_RxPacket, 8> FNT_RxFIFO;
       FIFO<FANET_Packet, 4> FNT_TxFIFO;
#endif

       uint8_t RX_OGN_Packets=0;            // [packets] counts received packets

       RFM_TRX           TRX;               // radio transceiver

       int32_t TX_Credit  = 0;              // [ms] counts transmitter time avoid using more than 1% or air time
       uint16_t RX_OGN_Count64=0;           // counts received packets for the last 64 seconds

