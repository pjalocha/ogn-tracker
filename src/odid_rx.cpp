#ifdef WITH_ODID_RX

#include <string.h>

#include <Arduino.h>
#include "esp_event.h"
#include "esp_wifi.h"
#include "odid.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "odid_rx.h"

static const uint8_t ODID_RX_Channel = 6;          // we listen for RemoteID on channel 6 of the 2.4GHz band
static const uint8_t ODID_RX_HeaderSize = 24;
static const uint16_t ODID_RX_MaxFrameSize = 512;  // [bytes]
static const uint8_t ODID_RX_QueueSize = 8;

enum ODID_RX_FrameKind : uint8_t
{ ODID_RX_FrameOther=0,
  ODID_RX_FrameBeacon=1,
  ODID_RX_FrameNAN=2,
  ODID_RX_FrameFrenchID=3 };

struct ODID_RX_QueuedFrame
{ uint16_t Length;
  uint8_t Kind;
  int8_t RSSI;
  uint8_t Frame[ODID_RX_MaxFrameSize];
};

static QueueHandle_t ODID_RX_Queue = NULL;
static SemaphoreHandle_t ODID_RX_ControlMutex = NULL;
static portMUX_TYPE ODID_RX_CounterMux = portMUX_INITIALIZER_UNLOCKED;
static ODID_RX_Counters ODID_RX_Counts={0,0,0,0};
static volatile uint32_t ODID_RX_Dropped = 0;
static bool ODID_RX_Initialized = false;
static bool ODID_RX_WifiStarted = false;

static bool ODID_RX_IsNANServiceId(const uint8_t *Data, uint16_t Length)
{ static const uint8_t ServiceId[6]={0x88,0x69,0x19,0x9D,0x92,0x09};
  uint16_t Offset=0;
  while(Offset+3<=Length)
  { const uint8_t Id=Data[Offset];
    const uint16_t AttrLen=(uint16_t)Data[Offset+1] | ((uint16_t)Data[Offset+2]<<8);
    Offset+=3;
    if(AttrLen>Length-Offset) return false;
    if(Id==0x02 && AttrLen>=6 && memcmp(Data+Offset, ServiceId, 6)==0) return true; // Service ID List
    if(Id==0x03 && AttrLen>=6 && memcmp(Data+Offset, ServiceId, 6)==0) return true; // Service Descriptor
    Offset+=AttrLen;
  }
  return false;
}

static ODID_RX_FrameKind ODID_RX_Classify(const uint8_t *Frame, uint16_t Length)
{ const uint16_t FrameControl=(uint16_t)Frame[0] | ((uint16_t)Frame[1]<<8);
  const uint16_t FrameType=FrameControl&0x00FC;
  const uint16_t End=(Length>=4) ? Length-4 : Length; // sig_len includes the FCS

  if(FrameType==0x0080) // Beacon
  { const uint16_t IEs=ODID_RX_HeaderSize+12; // fixed beacon fields
    if(End<IEs) return ODID_RX_FrameOther;
    uint16_t Offset=IEs;
    while(Offset+2<=End)
    { const uint8_t Id=Frame[Offset];
      const uint8_t IELen=Frame[Offset+1];
      Offset+=2;
      if(IELen>End-Offset) return ODID_RX_FrameOther;
      if(Id==221 && IELen>=4)
      { const uint8_t *Data=Frame+Offset;
        // FrenchID vendor IE: FRID_Packet::Read() expects the complete IE
        // (including 0xDD and its length) and checks the 6A:5C:35 signature.
        // Here validate the bounded IE before queuing it for PROC decoding.
        if(IELen>=3 && Data[0]==0x6A && Data[1]==0x5C && Data[2]==0x35)
          return ODID_RX_FrameFrenchID;
        if(Data[0]==0xFA && Data[1]==0x0B && Data[2]==0xBC && Data[3]==0x0D)
          return ODID_RX_FrameBeacon; // ASD-STAN Open Drone ID vendor IE
        if(Data[0]==0x50 && Data[1]==0x6F && Data[2]==0x9A && Data[3]==0x13 &&
           ODID_RX_IsNANServiceId(Data+4, IELen-4))
          return ODID_RX_FrameNAN; // NAN sync beacon advertising Remote ID service
      }
      Offset+=IELen;
    }
  }
  else if(FrameType==0x00D0 && End>=ODID_RX_HeaderSize+6) // Action
  { const uint8_t *Body=Frame+ODID_RX_HeaderSize;
    if(Body[0]==0x04 && Body[1]==0x09 && Body[2]==0x50 && Body[3]==0x6F &&
       Body[4]==0x9A && Body[5]==0x13 &&
       ODID_RX_IsNANServiceId(Body+6, End-(ODID_RX_HeaderSize+6)))
      return ODID_RX_FrameNAN; // NAN action carrying org.opendroneid.remoteid
  }
  return ODID_RX_FrameOther;
}

static void ODID_RX_PromiscuousCallback(void *Buffer, wifi_promiscuous_pkt_type_t Type)
{ if(!Buffer || Type!=WIFI_PKT_MGMT || !ODID_RX_Queue) return;

  const wifi_promiscuous_pkt_t *Packet=(const wifi_promiscuous_pkt_t *)Buffer;
  const uint8_t *Frame=Packet->payload;
  const uint16_t Length=Packet->rx_ctrl.sig_len;
  if(Packet->rx_ctrl.channel!=ODID_RX_Channel || Length<ODID_RX_HeaderSize) return;

  // Count group-addressed management frames; only verified Remote ID frames enter
  // the queue below.
  if((Frame[4]&0x01)==0) return;

  portENTER_CRITICAL(&ODID_RX_CounterMux);
  ODID_RX_Counts.All++;
  portEXIT_CRITICAL(&ODID_RX_CounterMux);

  const ODID_RX_FrameKind Kind=ODID_RX_Classify(Frame, Length);
  if(Kind==ODID_RX_FrameOther) return;

  portENTER_CRITICAL(&ODID_RX_CounterMux);
  if(Kind==ODID_RX_FrameBeacon) ODID_RX_Counts.Beacon++;
  else if(Kind==ODID_RX_FrameNAN) ODID_RX_Counts.Nan++;
  else if(Kind==ODID_RX_FrameFrenchID) ODID_RX_Counts.FrenchID++;
  portEXIT_CRITICAL(&ODID_RX_CounterMux);

  if(Length>ODID_RX_MaxFrameSize)
  { ODID_RX_Dropped++;
    return; }

  ODID_RX_QueuedFrame Queued;
  Queued.Length=Length;
  Queued.Kind=Kind;
  Queued.RSSI=Packet->rx_ctrl.rssi;
  memcpy(Queued.Frame, Frame, Length);

  if(xQueueSend(ODID_RX_Queue, &Queued, 0)!=pdTRUE)
    ODID_RX_Dropped++;
}

static void ODID_RX_PrintMac(const uint8_t *MAC)
{ Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                MAC[0], MAC[1], MAC[2], MAC[3], MAC[4], MAC[5]); }

static void ODID_RX_PrintBeaconRecords(const uint8_t *Frame, uint16_t Length)
{ const uint16_t End=(Length>=4) ? Length-4 : Length; // RX length includes FCS
  const uint16_t IEs=ODID_RX_HeaderSize+12;
  if(End<IEs) return;

  uint16_t Offset=IEs;
  while(Offset+2<=End)
  { const uint8_t Id=Frame[Offset];
    const uint8_t IELen=Frame[Offset+1];
    Offset+=2;
    if(IELen>End-Offset) return;
    if(Id==221 && IELen>=8)
    { const uint8_t *Data=Frame+Offset;
      if(Data[0]==0xFA && Data[1]==0x0B && Data[2]==0xBC && Data[3]==0x0D)
      { // Vendor data: OUI/type, message counter, then ODID message pack.
        const uint8_t Counter=Data[4];
        const uint8_t *Pack=Data+5;
        const uint16_t PackLen=IELen-5;
        if((Pack[0]>>4)!=0x0F || (Pack[0]&0x0F)!=2 || PackLen<3 || Pack[1]!=sizeof(ODID_Packet))
        { Serial.printf("RID ODID pack malformed counter=%u len=%u header=%02X\n",
                        Counter, PackLen, Pack[0]);
          return; }

        const uint8_t Records=Pack[2];
        const uint16_t Available=(PackLen-3)/sizeof(ODID_Packet);
        if(Records>Available)
        { Serial.printf("RID ODID pack truncated counter=%u records=%u available=%u\n",
                        Counter, Records, Available);
          return; }

        Serial.printf("RID ODID counter=%u records=%u\n", Counter, Records);
        for(uint8_t Idx=0; Idx<Records; Idx++)
        { ODID_Packet Packet;
          memcpy(&Packet, Pack+3+(uint16_t)Idx*sizeof(Packet), sizeof(Packet));
          if(!Packet.isValid())
          { Serial.printf("  ODID[%u] invalid header=%02X\n", Idx, Packet.Header);
            continue; }
          char RecordLine[160];
          const int LineLen=Packet.Print(RecordLine, sizeof(RecordLine));
          Serial.printf("  ODID[%u] %s: ", Idx, Packet.getPktTypeName());
          if(LineLen<0) Serial.print("format error");
          else if((size_t)LineLen>=sizeof(RecordLine)) Serial.print("record text truncated");
          else Serial.print(RecordLine);
          Serial.println(); }
        return;
      }
    }
    Offset+=IELen;
  }
}

static void ODID_RX_SetRxInfo(const ODID_RX_QueuedFrame &Frame,
                              OGN_RxPacket<OGN1_Packet> &RxPacket)
{ RxPacket.RxChan=ODID_RX_Channel;
  int16_t RSSI=-2*(int16_t)Frame.RSSI;                 // OGN receive RSSI is in half-dBm units
  if(RSSI<0) RSSI=0; else if(RSSI>255) RSSI=255;
  RxPacket.RxRSSI=(uint8_t)RSSI;
  RxPacket.RxErr=0;
  RxPacket.Correct=1;
  RxPacket.Warn=0;
  RxPacket.Rank=0;
  RxPacket.LatDist=0;
  RxPacket.LonDist=0; }

static bool ODID_RX_ConvertMessagePack(const uint8_t *Pack, uint16_t PackLen,
                                       const ODID_RX_QueuedFrame &Frame,
                                       OGN_RxPacket<OGN1_Packet> &RxPacket,
                                       int16_t GeoidSepar)
{ if(!Pack || PackLen<3 || (Pack[0]>>4)!=0x0F || (Pack[0]&0x0F)!=2 ||
     Pack[1]!=sizeof(ODID_Packet)) return false;
  const uint8_t Records=Pack[2];
  const uint16_t Available=(PackLen-3)/sizeof(ODID_Packet);
  if(Records>Available) return false;

  uint64_t MAC=0;
  for(uint8_t Idx=0; Idx<6; Idx++) MAC=(MAC<<8)|Frame.Frame[10+Idx]; // transmitter address
  for(uint8_t Idx=0; Idx<Records; Idx++)
  { ODID_Packet Record;
    memcpy(&Record, Pack+3+(uint16_t)Idx*sizeof(Record), sizeof(Record));
    if(!Record.isValid() || Record.Type!=ODID_Packet::PktType_Position) continue;
    if(!Record.Write(RxPacket.Packet, MAC, GeoidSepar)) continue;
    ODID_RX_SetRxInfo(Frame, RxPacket);
    return true; }
  return false;
}

static bool ODID_RX_ConvertMessagePack(const uint8_t *Pack, uint16_t PackLen,
                                       const ODID_RX_QueuedFrame &Frame,
                                       ADSL_Packet &RxPacket)
{ if(!Pack || PackLen<3 || (Pack[0]>>4)!=0x0F || (Pack[0]&0x0F)!=2 ||
     Pack[1]!=sizeof(ODID_Packet)) return false;
  const uint8_t Records=Pack[2];
  const uint16_t Available=(PackLen-3)/sizeof(ODID_Packet);
  if(Records>Available) return false;

  uint64_t MAC=0;
  for(uint8_t Idx=0; Idx<6; Idx++) MAC=(MAC<<8)|Frame.Frame[10+Idx]; // transmitter address
  for(uint8_t Idx=0; Idx<Records; Idx++)
  { ODID_Packet Record;
    memcpy(&Record, Pack+3+(uint16_t)Idx*sizeof(Record), sizeof(Record));
    if(!Record.isValid() || Record.Type!=ODID_Packet::PktType_Position) continue;
    if(!Record.Write(RxPacket, MAC)) continue;
    return true; }
  return false;
}

static void ODID_RX_SetRxInfo(const ODID_RX_QueuedFrame &Frame,
                              ADSL_RxPacket &RxPacket)
{ RxPacket.RxChan=ODID_RX_Channel;
  int16_t RSSI=-2*(int16_t)Frame.RSSI;                 // ADS-L receive RSSI is in half-dBm units
  if(RSSI<0) RSSI=0; else if(RSSI>255) RSSI=255;
  RxPacket.RxRSSI=(uint8_t)RSSI;
  RxPacket.RxErr=0;
  RxPacket.Correct=1;
  RxPacket.Warn=0;
  RxPacket.Rank=0;
  RxPacket.LatDist=0;
  RxPacket.LonDist=0; }

// Find and convert the first usable position in this one Beacon frame.
static bool ODID_RX_ConvertBeacon(const ODID_RX_QueuedFrame &Frame,
                                  ADSL_Packet &RxPacket,
                                  int16_t GeoidSepar)
{ (void)GeoidSepar; // OpenDroneID position altitude is already HAE.
  const uint16_t End=(Frame.Length>=4) ? Frame.Length-4 : Frame.Length;
  const uint16_t IEs=ODID_RX_HeaderSize+12;
  if(End<IEs) return false;

  uint16_t Offset=IEs;
  while(Offset+2<=End)
  { const uint8_t Id=Frame.Frame[Offset++];
    const uint8_t IELen=Frame.Frame[Offset++];
    const uint16_t DataOffset=Offset;
    if(IELen>End-DataOffset) return false;
    Offset+=IELen;
    if(Id!=221 || IELen<8) continue;

    const uint8_t *Data=Frame.Frame+DataOffset;
    if(Data[0]!=0xFA || Data[1]!=0x0B || Data[2]!=0xBC || Data[3]!=0x0D) continue;
    const uint8_t *Pack=Data+5;                      // skip OUI/type and message counter
    const uint16_t PackLen=IELen-5;
    if(ODID_RX_ConvertMessagePack(Pack, PackLen, Frame, RxPacket)) return true;
  }
  return false; }

// Find and convert the first usable position in this one Beacon frame.
static bool ODID_RX_ConvertBeacon(const ODID_RX_QueuedFrame &Frame,
                                  OGN_RxPacket<OGN1_Packet> &RxPacket,
                                  int16_t GeoidSepar)
{ const uint16_t End=(Frame.Length>=4) ? Frame.Length-4 : Frame.Length;
  const uint16_t IEs=ODID_RX_HeaderSize+12;
  if(End<IEs) return false;

  uint16_t Offset=IEs;
  while(Offset+2<=End)
  { const uint8_t Id=Frame.Frame[Offset++];
    const uint8_t IELen=Frame.Frame[Offset++];
    const uint16_t DataOffset=Offset;
    if(IELen>End-DataOffset) return false;
    Offset+=IELen;
    if(Id!=221 || IELen<8) continue;

    const uint8_t *Data=Frame.Frame+DataOffset;
    if(Data[0]!=0xFA || Data[1]!=0x0B || Data[2]!=0xBC || Data[3]!=0x0D) continue;
    const uint8_t *Pack=Data+5;                      // skip OUI/type and message counter
    const uint16_t PackLen=IELen-5;
    if(ODID_RX_ConvertMessagePack(Pack, PackLen, Frame, RxPacket, GeoidSepar)) return true;
  }
  return false;
}

static bool ODID_RX_ConvertNAN(const ODID_RX_QueuedFrame &Frame,
                               OGN_RxPacket<OGN1_Packet> &RxPacket,
                               int16_t GeoidSepar)
{ static const uint8_t NANHeader[6]={0x04, 0x09, 0x50, 0x6F, 0x9A, 0x13};
  static const uint8_t ServiceId[6]={0x88, 0x69, 0x19, 0x9D, 0x92, 0x09};
  const uint16_t End=(Frame.Length>=4) ? Frame.Length-4 : Frame.Length;
  if(End<ODID_RX_HeaderSize+sizeof(NANHeader)) return false;
  if(memcmp(Frame.Frame+ODID_RX_HeaderSize, NANHeader, sizeof(NANHeader))!=0) return false;

  // NAN action-frame body: service-discovery header, then TLV attributes.
  uint16_t Offset=ODID_RX_HeaderSize+sizeof(NANHeader);
  while(Offset+3<=End)
  { const uint8_t AttrId=Frame.Frame[Offset];
    const uint16_t AttrLen=(uint16_t)Frame.Frame[Offset+1] |
                           ((uint16_t)Frame.Frame[Offset+2]<<8);
    Offset+=3;
    const uint16_t AttrOffset=Offset;
    if(AttrLen>End-AttrOffset) return false;
    Offset+=AttrLen;
    if(AttrId!=0x03 || AttrLen<10) continue;           // Service Descriptor

    const uint8_t *Attr=Frame.Frame+AttrOffset;
    if(memcmp(Attr, ServiceId, sizeof(ServiceId))!=0 || Attr[8]!=0x10) continue;
    const uint8_t ServiceInfoLen=Attr[9];
    if(ServiceInfoLen<4 || (uint16_t)(10+ServiceInfoLen)!=AttrLen) continue;

    // Service Info is a one-byte message counter followed by the ODID pack.
    const uint8_t *Pack=Attr+11;
    const uint16_t PackLen=ServiceInfoLen-1;
    if(ODID_RX_ConvertMessagePack(Pack, PackLen, Frame, RxPacket, GeoidSepar)) return true;
  }
  return false;
}

static bool ODID_RX_ConvertNAN(const ODID_RX_QueuedFrame &Frame,
                               ADSL_Packet &RxPacket,
                               int16_t GeoidSepar)
{ (void)GeoidSepar; // OpenDroneID position altitude is already HAE.
  static const uint8_t NANHeader[6]={0x04, 0x09, 0x50, 0x6F, 0x9A, 0x13};
  static const uint8_t ServiceId[6]={0x88, 0x69, 0x19, 0x9D, 0x92, 0x09};
  const uint16_t End=(Frame.Length>=4) ? Frame.Length-4 : Frame.Length;
  if(End<ODID_RX_HeaderSize+sizeof(NANHeader)) return false;
  if(memcmp(Frame.Frame+ODID_RX_HeaderSize, NANHeader, sizeof(NANHeader))!=0) return false;

  uint16_t Offset=ODID_RX_HeaderSize+sizeof(NANHeader);
  while(Offset+3<=End)
  { const uint8_t AttrId=Frame.Frame[Offset];
    const uint16_t AttrLen=(uint16_t)Frame.Frame[Offset+1] |
                           ((uint16_t)Frame.Frame[Offset+2]<<8);
    Offset+=3;
    const uint16_t AttrOffset=Offset;
    if(AttrLen>End-AttrOffset) return false;
    Offset+=AttrLen;
    if(AttrId!=0x03 || AttrLen<10) continue;           // Service Descriptor

    const uint8_t *Attr=Frame.Frame+AttrOffset;
    if(memcmp(Attr, ServiceId, sizeof(ServiceId))!=0 || Attr[8]!=0x10) continue;
    const uint8_t ServiceInfoLen=Attr[9];
    if(ServiceInfoLen<4 || (uint16_t)(10+ServiceInfoLen)!=AttrLen) continue;

    const uint8_t *Pack=Attr+11;                       // skip service-info length and message counter
    const uint16_t PackLen=ServiceInfoLen-1;
    if(ODID_RX_ConvertMessagePack(Pack, PackLen, Frame, RxPacket)) return true;
  }
  return false;
}

static bool ODID_RX_ConvertFrenchID(const ODID_RX_QueuedFrame &Frame,
                                    OGN_RxPacket<OGN1_Packet> &RxPacket)
{ const uint16_t End=(Frame.Length>=4) ? Frame.Length-4 : Frame.Length;
  const uint16_t IEs=ODID_RX_HeaderSize+12;
  if(End<IEs) return false;

  uint64_t MAC=0;
  for(uint8_t Idx=0; Idx<6; Idx++) MAC=(MAC<<8)|Frame.Frame[10+Idx]; // transmitter address

  uint16_t Offset=IEs;
  while(Offset+2<=End)
  { const uint16_t IEOffset=Offset;
    const uint8_t Id=Frame.Frame[Offset++];
    const uint8_t IELen=Frame.Frame[Offset++];
    const uint16_t DataOffset=Offset;
    if(IELen>End-DataOffset) return false;
    Offset+=IELen;
    if(Id!=221 || IELen<3) continue;
    const uint8_t *Data=Frame.Frame+DataOffset;
    if(Data[0]!=0x6A || Data[1]!=0x5C || Data[2]!=0x35) continue;

    FRID_Packet Record;
    if(Record.Read(Frame.Frame+IEOffset)!=(int)IELen+2) continue;
    if(!Record.Write(RxPacket.Packet, MAC)) continue;
    ODID_RX_SetRxInfo(Frame, RxPacket);
    return true; }
  return false;
}

static bool ODID_RX_ConvertFrenchID(const ODID_RX_QueuedFrame &Frame,
                                    ADSL_Packet &RxPacket,
                                    int16_t GeoidSepar)
{ const uint16_t End=(Frame.Length>=4) ? Frame.Length-4 : Frame.Length;
  const uint16_t IEs=ODID_RX_HeaderSize+12;
  if(End<IEs) return false;

  uint64_t MAC=0;
  for(uint8_t Idx=0; Idx<6; Idx++) MAC=(MAC<<8)|Frame.Frame[10+Idx]; // transmitter address

  uint16_t Offset=IEs;
  while(Offset+2<=End)
  { const uint16_t IEOffset=Offset;
    const uint8_t Id=Frame.Frame[Offset++];
    const uint8_t IELen=Frame.Frame[Offset++];
    const uint16_t DataOffset=Offset;
    if(IELen>End-DataOffset) return false;
    Offset+=IELen;
    if(Id!=221 || IELen<3) continue;
    const uint8_t *Data=Frame.Frame+DataOffset;
    if(Data[0]!=0x6A || Data[1]!=0x5C || Data[2]!=0x35) continue;

    FRID_Packet Record;
    if(Record.Read(Frame.Frame+IEOffset)!=(int)IELen+2) continue;
    if(!Record.Write(RxPacket, MAC, GeoidSepar)) continue;
    return true; }
  return false;
}

int ODID_RX_GetPacket(ADSL_RxPacket &RxPacket, int16_t GeoidSepar)
{ if(!ODID_RX_Queue) return -1;
  RxPacket.Clear();
  RxPacket.RxChan=0;
  RxPacket.RxRSSI=0;
  static ODID_RX_QueuedFrame Frame;                  // keep the 512-byte frame off the PROC task stack
  if(xQueueReceive(ODID_RX_Queue, &Frame, 0)!=pdTRUE) return -1;
  bool Converted=false;
  if(Frame.Kind==ODID_RX_FrameBeacon)
    Converted=ODID_RX_ConvertBeacon(Frame, RxPacket.Packet, GeoidSepar);
  else if(Frame.Kind==ODID_RX_FrameFrenchID)
    Converted=ODID_RX_ConvertFrenchID(Frame, RxPacket.Packet, GeoidSepar);
  else if(Frame.Kind==ODID_RX_FrameNAN)
    Converted=ODID_RX_ConvertNAN(Frame, RxPacket.Packet, GeoidSepar);
  if(!Converted) return 0;
  ODID_RX_SetRxInfo(Frame, RxPacket);
  return 1; }

int ODID_RX_GetPacket(OGN_RxPacket<OGN1_Packet> &RxPacket, int16_t GeoidSepar)
{ if(!ODID_RX_Queue) return -1;
  RxPacket.Clear();
  RxPacket.RxChan=0;
  RxPacket.RxRSSI=0;
  static ODID_RX_QueuedFrame Frame;                  // keep the 512-byte frame off the PROC task stack
  if(xQueueReceive(ODID_RX_Queue, &Frame, 0)!=pdTRUE) return -1;
  if(Frame.Kind==ODID_RX_FrameBeacon)
    return ODID_RX_ConvertBeacon(Frame, RxPacket, GeoidSepar) ? 1 : 0;
  if(Frame.Kind==ODID_RX_FrameFrenchID)
    return ODID_RX_ConvertFrenchID(Frame, RxPacket) ? 1 : 0;
  if(Frame.Kind==ODID_RX_FrameNAN)
    return ODID_RX_ConvertNAN(Frame, RxPacket, GeoidSepar) ? 1 : 0;
  return 0; }

void ODID_RX_ProcessPending(void)
{ if(!ODID_RX_Queue) return;
  static uint32_t ReportedDrops=0;
  ODID_RX_QueuedFrame Queued;
  while(xQueueReceive(ODID_RX_Queue, &Queued, 0)==pdTRUE)
  { const uint16_t FrameControl=(uint16_t)Queued.Frame[0] | ((uint16_t)Queued.Frame[1]<<8);
    const uint8_t Type=(FrameControl>>2)&0x03;
    const uint8_t Subtype=(FrameControl>>4)&0x0F;
    const uint16_t Sequence=((uint16_t)Queued.Frame[22] | ((uint16_t)Queued.Frame[23]<<8))>>4;
    const char *Kind=Queued.Kind==ODID_RX_FrameBeacon ? "BCN" :
                     Queued.Kind==ODID_RX_FrameNAN ? "NAN" : "FID";
    Serial.printf("RID %s #%u %ddBm %uB:[%04X:%u:%u:%u] DA=",
                  Kind, ODID_RX_Channel, Queued.RSSI, Queued.Length,
                  FrameControl, Type, Subtype, Sequence);
    ODID_RX_PrintMac(Queued.Frame+4);
    Serial.print(" SA="); ODID_RX_PrintMac(Queued.Frame+10);
    Serial.print(" BSSID="); ODID_RX_PrintMac(Queued.Frame+16);
    Serial.println();
    if(Queued.Kind==ODID_RX_FrameBeacon)
      ODID_RX_PrintBeaconRecords(Queued.Frame, Queued.Length); }
  const uint32_t Drops=ODID_RX_Dropped;
  if(Drops!=ReportedDrops)
  { Serial.printf("WiFi Remote ID queue dropped=%lu total\n", (unsigned long)Drops);
    ReportedDrops=Drops; }
}

void ODID_RX_GetCounters(ODID_RX_Counters *Counters)
{ if(!Counters) return;
  portENTER_CRITICAL(&ODID_RX_CounterMux);
  *Counters=ODID_RX_Counts;
  portEXIT_CRITICAL(&ODID_RX_CounterMux); }

esp_err_t ODID_RX_Start(void)
{ if(!ODID_RX_Initialized || !ODID_RX_ControlMutex) return ESP_ERR_INVALID_STATE;
  if(xSemaphoreTake(ODID_RX_ControlMutex, portMAX_DELAY)!=pdTRUE) return ESP_FAIL;
  if(ODID_RX_WifiStarted)
  { xSemaphoreGive(ODID_RX_ControlMutex);
    return ESP_OK; }

  esp_err_t Err=esp_wifi_start();
  if(Err==ESP_OK) Err=esp_wifi_set_channel(ODID_RX_Channel, WIFI_SECOND_CHAN_NONE);
  if(Err==ESP_OK) Err=esp_wifi_set_promiscuous(true);
  if(Err!=ESP_OK)
  { esp_wifi_set_promiscuous(false);
    esp_wifi_stop(); }
  else
  { ODID_RX_WifiStarted=true;
    Serial.printf("WiFi RX sniffer started: mode=NULL channel=%u filter=group management frames\n",
                  ODID_RX_Channel); }

  xSemaphoreGive(ODID_RX_ControlMutex);
  return Err;
}

esp_err_t ODID_RX_Stop(void)
{ if(!ODID_RX_Initialized || !ODID_RX_ControlMutex) return ESP_ERR_INVALID_STATE;
  if(xSemaphoreTake(ODID_RX_ControlMutex, portMAX_DELAY)!=pdTRUE) return ESP_FAIL;
  if(!ODID_RX_WifiStarted)
  { xSemaphoreGive(ODID_RX_ControlMutex);
    return ESP_OK; }

  esp_err_t PromiscuousErr=esp_wifi_set_promiscuous(false);
  esp_err_t StopErr=esp_wifi_stop();
  if(StopErr==ESP_OK) ODID_RX_WifiStarted=false;
  xSemaphoreGive(ODID_RX_ControlMutex);
  if(StopErr!=ESP_OK) return StopErr;
  if(PromiscuousErr!=ESP_OK) return PromiscuousErr;
  Serial.println("WiFi RX sniffer stopped");
  return ESP_OK;
}

esp_err_t ODID_RX_Init(void)
{ ODID_RX_Queue=xQueueCreate(ODID_RX_QueueSize, sizeof(ODID_RX_QueuedFrame));
  if(!ODID_RX_Queue) return ESP_ERR_NO_MEM;
  ODID_RX_ControlMutex=xSemaphoreCreateMutex();
  if(!ODID_RX_ControlMutex) return ESP_ERR_NO_MEM;

  esp_err_t Err=esp_event_loop_create_default();
  if(Err!=ESP_OK && Err!=ESP_ERR_INVALID_STATE) return Err;

#ifndef WITH_WIFI
  wifi_init_config_t Config=WIFI_INIT_CONFIG_DEFAULT();
  Err=esp_wifi_init(&Config); if(Err!=ESP_OK) return Err;
#endif

  Err=esp_wifi_set_storage(WIFI_STORAGE_RAM); if(Err!=ESP_OK) return Err;
  Err=esp_wifi_set_mode(WIFI_MODE_NULL); if(Err!=ESP_OK) return Err;
  Err=esp_wifi_set_promiscuous_rx_cb(ODID_RX_PromiscuousCallback); if(Err!=ESP_OK) return Err;

  wifi_promiscuous_filter_t Filter={ .filter_mask=WIFI_PROMIS_FILTER_MASK_MGMT };
  Err=esp_wifi_set_promiscuous_filter(&Filter); if(Err!=ESP_OK) return Err;

  ODID_RX_Initialized=true;
  return ODID_RX_Start();
}

#endif // WITH_ODID_RX
