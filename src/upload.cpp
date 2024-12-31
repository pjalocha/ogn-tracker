#include "main.h"

#ifdef WITH_WIFI

#include "socket.h"
#include "wifi.h"
#include "log.h"
#include "gps.h"

#define DEBUG_PRINT

static wifi_ap_record_t AP[8];                        // lists of Access Points from the WiFi scan
static uint16_t APs=0;

static const int MaxLineLen=1024;
static char Line[MaxLineLen];                         // for printing and buffering

static void AP_Print(void (*Output)(char))            // print lists of AP's
{ for(uint16_t Idx=0; Idx<APs; Idx++)
  { Line[0]='0'+Idx; Line[1]=':'; Line[2]=' ';
    uint8_t Len=3+AP_Print(Line+3, AP+Idx);
    Format_String(Output, Line); }
}

// ----------------------------------------------------------------------------------------------

#include "esp_http_client.h"

static esp_err_t HTTP_event_handler(esp_http_client_event_t *evt)
{ switch (evt->event_id)
  { case HTTP_EVENT_ON_DATA:
      ESP_LOGI(TAG, "Data received: %.*s", evt->data_len, (char *)evt->data);
      break;
    default:
      break;
  }
  return ESP_OK; }

// static const char *UploadURL = "http://ogn3.glidernet.org:8084/upload";

/*
void send_chunk(esp_http_client_handle_t client, const char *data, size_t len)
{ char chunk_header[16];
  snprintf(chunk_header, sizeof(chunk_header), "%X\r\n", len);  // Hex length with CRLF

  esp_http_client_write(client, chunk_header, strlen(chunk_header));  // Send chunk size
  esp_http_client_write(client, data, len);                           // Send chunk data
  esp_http_client_write(client, "\r\n", 2); }                         // End of chunk
*/

static int UploadFile(const char *LocalFileName, const char *RemoteFileName)
{ FILE *File = fopen(LocalFileName, "rb"); if(File==0) return -1;

  fseek(File, 0, SEEK_END);
  int FileSize = ftell(File);
  rewind(File);

  // esp_http_client_config_t Config =
  // { .url = Parameters.UploadURL,
  //   .event_handler = HTTP_event_handler };

  esp_http_client_config_t Config =
  { .url = Parameters.UploadURL,
    .method = HTTP_METHOD_POST,
    .event_handler = NULL };

  esp_http_client_handle_t Client = esp_http_client_init(&Config);
  // esp_http_client_set_method(Client, HTTP_METHOD_POST);
  esp_http_client_set_header(Client, "Content-Type", "application/octet-stream");
  esp_http_client_set_header(Client, "X-File-Name", RemoteFileName);

  esp_err_t Err = esp_http_client_open(Client, FileSize); // start the HTTP request: -1 means chunked transfer
  if(Err!=ESP_OK)
  { fclose(File);
    esp_http_client_cleanup(Client);
    return -2; }

  int SendSize=0;
  for( ; ; )
  { int Read=fread(Line, 1, MaxLineLen, File); if(Read<=0) break;
    SendSize+=Read;
    Err = esp_http_client_write(Client, Line, Read);
    if(Err<0) break; }

  fclose(File);

  int ContLen = esp_http_client_fetch_headers(Client);
  int StatCode=0;
  int RespLen=0;
  if(ContLen>0)
  { StatCode = esp_http_client_get_status_code(Client);
    RespLen = esp_http_client_read_response(Client, Line, MaxLineLen); }

  esp_http_client_close(Client);
  esp_http_client_cleanup(Client);
  return Err>=0 && StatCode==200 ? SendSize:-3; }

static char LocalFile[64];
static char RemoteFile[128];

static int RemoteLogFileName(char *Name, uint32_t Time)
{ uint64_t ID=getUniqueID();
  int Len=Format_Hex(Name, (uint16_t)(ID>>32)); Len+=Format_Hex(Name+Len, (uint32_t)ID);
  Name[Len++]='_';
  Len+=Format_Hex(Name+Len, Parameters.AcftID);
  Name[Len++]='_';
  Len+=FlashLog_ShortFileName(Name+Len, Time);
  return Len; }

static int UploadOldestFile(bool Delete=1)
{ uint32_t Oldest=0;
  int Files=FlashLog_FindOldestFile(Oldest);
  if(Files==0 || Oldest==0) return 0;
  FlashLog_FullFileName(LocalFile, Oldest);                  // local file name, including the path
  RemoteLogFileName(RemoteFile, Oldest);                     // remote file name, including tracker MAC and ID
  int Err=UploadFile(LocalFile, RemoteFile);
  sprintf(Line, "Upload: %s => %s => %d\n", LocalFile, RemoteFile, Err);
  Format_String(CONS_UART_Write, Line);
  if(Err>=0 && Delete) remove(LocalFile);
  return Err; }

static int Upload(void)
{ int Err=0;
  for( ; ; )
  { if(Parameters.UploadURL[0]==0) break;
    Err=UploadOldestFile(); if(Err<=0) break; }
  return 0; }

// ----------------------------------------------------------------------------------------------

#ifdef UPLOAD_TO_SOCKET
static const char *UploadHost = "ogn3.glidernet.org";                // server address
static const char *UploadPort = "8084";                              // server socket

static Socket UploadSocket;                                          // socket to talk to the server

static int UploadDialog(void)               // connect and talk to the server exchaging data
{ int ConnErr=UploadSocket.Connect(UploadHost, UploadPort);   // connect to the server
  if(ConnErr>=0)                                           // if connection succesfull
  { strcpy(Line, "OGN-Tracker\n");
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Connected to ");
    IP_Print(CONS_UART_Write, UploadSocket.getIP());
    Format_String(CONS_UART_Write, "\nUpload <- ");
    Format_String(CONS_UART_Write, Line);
    xSemaphoreGive(CONS_Mutex);
#endif
    UploadSocket.setReceiveTimeout(1);                      // [sec] receive timeout
    // UploadSocket.setBlocking(0);
    int Write=UploadSocket.Send(Line, strlen(Line));        // send login to the server
    vTaskDelay(1000);
    UploadSocket.Disconnect();
  }
#ifdef DEBUG_PRINT
  else                                                      // if connection failed to the server
  { xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Failed to connect to ");
    IP_Print(CONS_UART_Write, UploadSocket.getIP());
    Format_String(CONS_UART_Write, " -> ");
    Format_SignDec(CONS_UART_Write, ConnErr);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex); }
#endif
  UploadSocket.Disconnect();
  return 0; }
#endif

// ----------------------------------------------------------------------------------------------

extern "C"
void vTaskUPLOAD(void* pvParameters)
{
  vTaskDelay(10000);

  for( ; ; )
  { vTaskDelay(60000);
    if(Parameters.UploadURL[0]==0) continue;                         // don't upload if URL not defined
    if(Flight.inFlight()) continue;                                  // don't unload if airborne
    if(FlashLog_isOpen()) continue;                                  // don't upload if a log file is being written

    uint32_t Oldest=0;
    int Files=FlashLog_FindOldestFile(Oldest);
    if(Files==0 || Oldest==0) continue;                              // don't attempt to upload if no log files

    esp_err_t Err=WIFI_Start();                                      // start WiFi in station-mode
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_Start() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif

    vTaskDelay(1000);
    APs=8;
    Err=WIFI_PassiveScan(AP, APs);                           // perform a passive scan: find Access Points around
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_PassiveScan() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    CONS_UART_Write('/');
    Format_UnsDec(CONS_UART_Write, APs);
    Format_String(CONS_UART_Write, "\n");
    if(Err==ESP_OK) AP_Print(CONS_UART_Write);
    xSemaphoreGive(CONS_Mutex);
#endif

    if(Err==ESP_OK)                                           // if WiFi scan went well
    { for(uint16_t Idx=0; Idx<APs; Idx++)                     // loop over Access Points
      { const char *NetName  = (const char *)(AP[Idx].ssid);
        const char *NetPass = 0;
        if(AP[Idx].authmode!=WIFI_AUTH_OPEN)                  // if not an open network
        { NetPass=Parameters.getWIFIpass(NetName);            // then search the password
          if(NetPass==0) continue; }                          // give up if no password for this network
        Err=WIFI_Connect(AP+Idx, NetPass);
#ifdef DEBUG_PRINT
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, NetName);
        if(NetPass) { CONS_UART_Write('/'); Format_String(CONS_UART_Write, NetPass); }
        CONS_UART_Write(':');
        CONS_UART_Write(' ');
        Format_String(CONS_UART_Write, "WIFI_Connect() => ");
        if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
        Format_SignDec(CONS_UART_Write, Err);
        Format_String(CONS_UART_Write, "\n");
        xSemaphoreGive(CONS_Mutex);
#endif
        if(Err) continue;                                     // if connection failed then give up ad move on to the next AP

        WIFI_IP.ip.addr = 0; WIFI_IP.gw.addr=0;
        for(uint8_t Idx=0; Idx<10; Idx++)                     // wait to obtain local IP from DHCP
        { vTaskDelay(1000);
          // if(WIFI_State.isConnected==1) break;
          if(WIFI_getLocalIP())
          { if(WIFI_IP.ip.addr && WIFI_IP.gw.addr) break; }
        }

#ifdef DEBUG_PRINT
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "Local IP: ");
        IP_Print(CONS_UART_Write, WIFI_IP.ip.addr);
        Format_String(CONS_UART_Write, " GW: ");
        IP_Print(CONS_UART_Write, WIFI_IP.gw.addr);
        Format_String(CONS_UART_Write, "\n");
        xSemaphoreGive(CONS_Mutex);
#endif
        if(WIFI_IP.ip.addr==0) { WIFI_Disconnect(); continue; }     // if getting local IP failed then give up and try another AP

        Upload();

        vTaskDelay(2000);
        WIFI_Disconnect(); WIFI_IP.ip.addr=0;
        vTaskDelay(2000);
      }
    }

    vTaskDelay(2000);
    Err=WIFI_Stop();
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_Stop() => ");
    if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif

  }
}

#endif // WITH_WIFI
