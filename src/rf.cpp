#include <stdint.h>

#include "rf.h"
#include "gps.h"
#include "timesync.h"

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

       int32_t TX_Credit  = 0;              // [ms] counts transmitter time avoid using more than 1% or air time
       uint16_t RX_OGN_Count64=0;           // counts received packets for the last 64 seconds

       RFM_TRX           TRX;

// OGNv1 SYNC:       0x0AF3656C encoded in Manchester
static const uint8_t OGN1_SYNC[10] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A, 0x00, 0x00 };
static const uint8_t *OGN_SYNC = OGN1_SYNC;
// ADS-L SYNC:       0xF5724B18 encoded in Manchester (fixed packet length 0x18 is included)
static const uint8_t ADSL_SYNC[10] = { 0x55, 0x99, 0x95, 0xA6, 0x9A, 0x65, 0xA9, 0x6A, 0x00, 0x00 };
// PilotAware SYNC, includes net-address which is always zero, and the packet size which is always 0x18 = 24
static const uint8_t PAW_SYNC [10] = { 0xB4, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x18, 0x71, 0x00, 0x00 };

 void vTaskRF(void* pvParameters)
{
  RF_RxFIFO.Clear();                      // clear receive/transmit packet FIFO's
  RF_TxFIFO.Clear();
#ifdef WITH_FANET
  FNT_RxFIFO.Clear();
  FNT_TxFIFO.Clear();
#endif
  TRX.Init();

  RF_FreqPlan.setPlan(Parameters.FreqPlan);     // 1 = Europe/Africa, 2 = USA/CA, 3 = Australia and South America
  TRX.BaseFrequency = RF_FreqPlan.BaseFreq;
  TRX.ChannelSpacing = RF_FreqPlan.ChanSepar;
  TRX.FreqCorr = Parameters.RFchipFreqCorr;

  vTaskDelay(1000);

  for( ; ; )
  {
    for( ; TimeSync_msTime()<400; )
    { vTaskDelay(1); }
    TRX.ConfigManchFSK(26, OGN_SYNC, 8);                         // configure for OGN
    TRX.setOutputPower(Parameters.TxPower);
    TRX.setChannel(1);
    const OGN_TxPacket<OGN_Packet> *TxPkt0 = RF_TxFIFO.getRead();
    if(TxPkt0)
    { TRX.TxManchFSK(TxPkt0->Byte(), 26);
      RF_TxFIFO.Read(); }

    // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
    // Format_String(CONS_UART_Write, "RF: Slot 0\n");
    // xSemaphoreGive(CONS_Mutex);                                  // give back UART1 to other tasks

    vTaskDelay(300);

    for( ; TimeSync_msTime()<800; )
    { vTaskDelay(1); }
    TRX.setChannel(0);
    const OGN_TxPacket<OGN_Packet> *TxPkt1 = RF_TxFIFO.getRead();
    if(TxPkt1)
    { TRX.TxManchFSK(TxPkt1->Byte(), 26);
      RF_TxFIFO.Read(); }
    else if(TxPkt0)
      TRX.TxManchFSK(TxPkt0->Byte(), 26);

    // xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
    // Format_String(CONS_UART_Write, "RF: Slot 1\n");
    // xSemaphoreGive(CONS_Mutex);                                  // give back UART1 to other tasks

    vTaskDelay(400);

  }

}
