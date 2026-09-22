#if defined(WITH_MOBILE) && defined(WITH_WIFI)

#include <lwip/inet.h>

#include "mobile.h"
#include "gdl90.h"
#include "socket.h"
#include "wifi.h"

static const uint16_t MOBILE_ServerPort = 14590;
static const char    *MOBILE_ServerAddr = "51.178.37.174";  // ogn3.glidernet.org
static const uint8_t  MOBILE_ADSL_ID    = 0x60;             // GDL90 ID for ADS-L packets
static const uint8_t  MOBILE_OGN_ID     = 0x5F;             // GDL90 ID for OGN1 packets
// static const uint8_t  MOBILE_FLR_ID     = 0x5E;             // GDL90 ID for FLARM packets - but we could send ADS-L packets instead

static const uint16_t MOBILE_TxUDPsizeMax = 1024;

static UDPsocket MOBILE_Socket;
static uint32_t  MOBILE_ServerIP = 0;
static bool      MOBILE_Ready = false;
static uint8_t   MOBILE_TxUDP[MOBILE_TxUDPsizeMax];
static uint16_t  MOBILE_TxUDPsize = 0;

void MOBILE_Init(void)
{ if(MOBILE_Ready) return;
  MOBILE_ServerIP = inet_addr(MOBILE_ServerAddr);
  if(MOBILE_ServerIP==IPADDR_NONE) return;
  if(MOBILE_Socket.Bind(0)<0) return;                  // use an ephemeral local port for replies
  if(MOBILE_Socket.setNonBlocking()!=0) { MOBILE_Socket.Close(); return; }
  MOBILE_Ready=true; }

bool MOBILE_Flush(void)
{ if(MOBILE_TxUDPsize==0) return true;
  if(!WIFI_isConnected()) { MOBILE_TxUDPsize=0; return false; }
  if(!MOBILE_Ready) MOBILE_Init();
  if(!MOBILE_Ready) { MOBILE_TxUDPsize=0; return false; }
  bool Sent=MOBILE_Socket.SendTo(MOBILE_ServerIP, MOBILE_ServerPort,
                                 MOBILE_TxUDP, MOBILE_TxUDPsize)==MOBILE_TxUDPsize;
  MOBILE_TxUDPsize=0;
  return Sent; }

static bool MOBILE_QueueGDL90(uint8_t ID, const uint8_t *Data, int Len)
{ if(!WIFI_isConnected()) return false;
  if(!MOBILE_Ready) MOBILE_Init();
  if(!MOBILE_Ready) return false;

  // GDL90_Send() can double every byte which needs escaping.  The two flags
  // are not escaped, so this is a safe upper bound for one complete frame.
  const uint16_t MaxFrameSize=2+2*(Len+3);
  if(MaxFrameSize>MOBILE_TxUDPsizeMax) return false;
  if(MOBILE_TxUDPsize+MaxFrameSize>MOBILE_TxUDPsizeMax)
    MOBILE_Flush();
  if(MOBILE_TxUDPsize+MaxFrameSize>MOBILE_TxUDPsizeMax) return false;

  int FrameSize=GDL90_Send(MOBILE_TxUDP+MOBILE_TxUDPsize, ID, Data, Len, 1);  // escape all control characters
  MOBILE_TxUDPsize+=FrameSize;
  return FrameSize>0; }

bool MOBILE_SendADSL(const ADSL_Packet &Packet)
{ const int Bytes=ADSL_Packet::TxBytes-3-3;                // Version and 20-byte body, no ADS-L CRC
  return MOBILE_QueueGDL90(MOBILE_ADSL_ID, &Packet.Version, Bytes); }

bool MOBILE_SendOGN(const OGN1_Packet &Packet)
{ const int Bytes=OGN1_Packet::Bytes;                         // omit the 6-byte OGN FEC
  return MOBILE_QueueGDL90(MOBILE_OGN_ID, Packet.Byte(), Bytes); }

#endif // WITH_MOBILE && WITH_WIFI
