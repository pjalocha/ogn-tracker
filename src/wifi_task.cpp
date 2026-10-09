#ifdef WITH_WIFI

#include <string.h>

#include "main.h"
#include "wifi.h"

static const uint8_t WIFI_ScanMax = 16;

static void WIFI_Report(const char *Message)
{
  if(!xSemaphoreTake(CONS_Mutex, 100)) return;
  Format_String(CONS_UART_Write, "WIFI: ");
  Format_String(CONS_UART_Write, Message);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
}

static void WIFI_ReportAP(const char *Action, const char *SSID)
{
  if(!xSemaphoreTake(CONS_Mutex, 100)) return;
  Format_String(CONS_UART_Write, "WIFI: ");
  Format_String(CONS_UART_Write, Action);
  Format_String(CONS_UART_Write, " '");
  Format_String(CONS_UART_Write, SSID);
  Format_String(CONS_UART_Write, "'\n");
  xSemaphoreGive(CONS_Mutex);
}

static void WIFI_ReportError(const char *Action, esp_err_t Err)
{
  if(!xSemaphoreTake(CONS_Mutex, 100)) return;
  Format_String(CONS_UART_Write, "WIFI: ");
  Format_String(CONS_UART_Write, Action);
  Format_String(CONS_UART_Write, " error ");
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
}

static const char *WIFI_FindPassword(const char *SSID)
{
  for(uint8_t Idx=0; Idx<Parameters.WIFIsets; Idx++)
  { if(Parameters.WIFIname[Idx][0] && strcmp(Parameters.WIFIname[Idx], SSID)==0)
      return Parameters.WIFIpass[Idx]; }
  return 0;
}

static bool WIFI_ConnectAndWait(wifi_ap_record_t *AP, const char *Pass)
{
  WIFI_ReportAP("trying AP", (const char *)AP->ssid);
  xSemaphoreTake(WIFI_Mutex, portMAX_DELAY);
  esp_err_t Err=WIFI_Connect(AP, Pass);
  xSemaphoreGive(WIFI_Mutex);
  if(Err!=ESP_OK)
  { WIFI_ReportError("connection request", Err);
    return false; }

  WIFI_IP.ip.addr=0;
  WIFI_IP.gw.addr=0;
  for(uint8_t Retry=0; Retry<30; Retry++)
  { vTaskDelay(pdMS_TO_TICKS(500));
    if(WIFI_getLocalIP())
    { if(!xSemaphoreTake(CONS_Mutex, 100)) return true;
      Format_String(CONS_UART_Write, "WIFI: connected to '");
      Format_String(CONS_UART_Write, (const char *)AP->ssid);
      Format_String(CONS_UART_Write, "', IP: ");
      IP_Print(CONS_UART_Write, WIFI_IP.ip.addr);
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex);
      return true; }
    if(WIFI_State.isConnected==1) break;
  }

  xSemaphoreTake(WIFI_Mutex, portMAX_DELAY);
  WIFI_Disconnect();
  xSemaphoreGive(WIFI_Mutex);
  WIFI_IP.ip.addr=0;
  WIFI_IP.gw.addr=0;
  WIFI_ReportAP("AP connection failed (no IP)", (const char *)AP->ssid);
  return false;
}

static bool WIFI_TryConfigured(wifi_ap_record_t *AP, uint16_t APs)
{
  bool Found=false;
  for(uint16_t Idx=0; Idx<APs; Idx++)
  { const char *SSID=(const char *)AP[Idx].ssid;
    const char *Pass=WIFI_FindPassword(SSID);
    if(!Pass) continue;
    Found=true;
    if(WIFI_ConnectAndWait(AP+Idx, Pass)) return true;
  }
  if(!Found) WIFI_Report("no configured AP found in scan");
  return false;
}

static bool WIFI_TryOpen(wifi_ap_record_t *AP, uint16_t APs)
{
  bool Found=false;
  for(uint16_t Idx=0; Idx<APs; Idx++)
  { if(AP[Idx].authmode!=WIFI_AUTH_OPEN) continue;
    Found=true;
    if(WIFI_ConnectAndWait(AP+Idx, 0)) return true;
  }
  if(!Found) WIFI_Report("no open AP found in scan");
  return false;
}

bool WIFI_WaitForConnection(uint32_t TimeoutMs)
{
  const uint32_t Start=millis();
  for( ; ; )
  { if(WIFI_getLocalIP()) return true;
    if((uint32_t)(millis()-Start)>=TimeoutMs) return false;
    vTaskDelay(pdMS_TO_TICKS(250)); }
}

extern "C"
void vTaskWIFI(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(1000));
  WIFI_Report("manager started");
  bool Started=false;
  bool WasConnected=false;

  for( ; ; )
  { if(WIFI_isConnected())
    { if(!WasConnected) WasConnected=true;
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue; }
    if(WasConnected)
    { WIFI_Report("connection lost, restarting search");
      WasConnected=false; }

    esp_err_t Err=ESP_OK;
    if(!Started)
    { xSemaphoreTake(WIFI_Mutex, portMAX_DELAY);
      Err=WIFI_Start();
      xSemaphoreGive(WIFI_Mutex);
      if(Err!=ESP_OK && Err!=ESP_ERR_WIFI_STATE)
      { WIFI_ReportError("station start", Err);
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue; }
      WIFI_Report("station mode started");
      Started=true;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    wifi_ap_record_t AP[WIFI_ScanMax];
    uint16_t APs=WIFI_ScanMax;
    xSemaphoreTake(WIFI_Mutex, portMAX_DELAY);
    Err=WIFI_PassiveScan(AP, APs);
    xSemaphoreGive(WIFI_Mutex);

    if(Err!=ESP_OK)
      WIFI_ReportError("scan", Err);
    else
    { if(APs==0) WIFI_Report("scan found no APs");
      else
      { if(xSemaphoreTake(CONS_Mutex, 100))
        { Format_String(CONS_UART_Write, "WIFI: scan found ");
          Format_UnsDec(CONS_UART_Write, APs);
          Format_String(CONS_UART_Write, " AP(s)\n");
          xSemaphoreGive(CONS_Mutex); }
        if(WIFI_TryConfigured(AP, APs) || WIFI_TryOpen(AP, APs))
          continue;
      }
      WIFI_Report("no usable AP; retrying");
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

#endif // WITH_WIFI
