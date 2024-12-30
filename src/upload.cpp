#include "main.h"

#ifdef WITH_WIFI

#include "socket.h"
#include "wifi.h"

static Socket UploadSocket;                                          // socket to talk to the server

static const char *UploadHost = "aprs.glidernet.org";                // server address
static const char *UploadPort = "14580";                             // server socket

static wifi_ap_record_t AP[8];                        // lists of Access Points from the WiFi scan
static uint16_t APs=0;

static const int MaxLineLen=512;
static char Line[MaxLineLen];                         // for printing and buffering

static void AP_Print(void (*Output)(char))            // print lists of AP's
{ for(uint16_t Idx=0; Idx<APs; Idx++)
  { Line[0]='0'+Idx; Line[1]=':'; Line[2]=' ';
    uint8_t Len=3+AP_Print(Line+3, AP+Idx);
    Format_String(Output, Line); }
}

#define DEBUG_PRINT

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


extern "C"
void vTaskUPLOAD(void* pvParameters)
{
  vTaskDelay(10000);

  for( ; ; )
  { vTaskDelay(10000);
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

        UploadDialog();

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
