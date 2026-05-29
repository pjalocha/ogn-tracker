#ifndef __SDLOG_H__
#define __SDLOG_H__

#include <stdint.h>

#include "main.h"
#include "fifo.h"

#ifdef WITH_SDLOG
#include "igc-key.h"
#endif

void Log_Write(char Byte);
int  Log_Free(void);
extern SemaphoreHandle_t Log_Mutex;

extern FIFO<OGN_RxPacket<OGN_Packet>, 16> IGClog_OGN_FIFO;
extern FIFO<ADSL_RxPacket           , 16> IGClog_ADSL_FIFO;

#ifdef WITH_SDLOG
extern IGC_Key IGC_SignKey;
#endif

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskSDLOG(void* pvParameters);

/*
#ifdef __cplusplus
  extern "C"
#endif
 void vTaskIGC(void* pvParameters);
*/
#endif // __SDLOG_H__
