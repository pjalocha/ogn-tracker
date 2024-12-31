#include "fifo.h"
#include "ogn.h"

extern FIFO<OGN_LogPacket<OGN_Packet>, 32> FlashLog_FIFO;

extern const char *SDcard_Path;      // with sub-directory which is created if does not exist
extern const char *FlashLog_Path;    // path to log files
extern const char *FlashLog_Ext;     // extension for log files, could be as well .TLA

void AddPath(char *Name, const char *FileName, const char *Path);

extern bool     FlashLog_SaveReq;
extern uint32_t FlashLog_FileTime;                            // [sec] start time of the current log file
extern char     FlashLog_FileName[32];                        // current log file name if open

bool FlashLog_isOpen(void);
int  FlashLog_FullFileName(char *FileName, uint32_t Time);    // create full name (including the path) of the log file starting from Time
int  FlashLog_ShortFileName(char *FileName, uint32_t Time);
uint32_t FlashLog_ReadShortFileTime(const char *FileName, int Len);
uint32_t FlashLog_ReadShortFileTime(const char *FileName);
int  FlashLog_CopyToSD(bool Remove=0);
int  FlashLog_FindOldestFile(uint32_t &Oldest, uint32_t After=0);        // find the oldest log file
int  FlashLog_ListFiles(void);                                    // list the log files on the console
int  FlashLog_ListFile(uint32_t FileTime);                        //
int  FlashLog_ListFile(const char *FileName, uint32_t FileTime);  //
// int  FlashLog_ListFile(const char *FileName, int Len);         //

#ifdef __cplusplus
  extern "C"
#endif
 void vTaskLOG(void* pvParameters);

