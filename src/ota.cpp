#include "main.h"

#ifdef WITH_OTA
#ifdef WITH_WIFI

#include "socket.h"
#include "wifi.h"
#include "log.h"
#include "gps.h"

#include "esp_check.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "nvs.h"
#include "nvs_flash.h"

#define DEBUG_PRINT

static wifi_ap_record_t AP[8]; // lists of Access Points from the WiFi scan
static uint16_t APs = 0;

static const int MaxLineLen = 1024;
static char Line[MaxLineLen]; // for printing and buffering

static void AP_Print(void (*Output)(char)) // print lists of AP's
{
  for (uint16_t Idx = 0; Idx < APs; Idx++)
  {
    Line[0] = '0' + Idx;
    Line[1] = ':';
    Line[2] = ' ';
    uint8_t Len = 3 + AP_Print(Line + 3, AP + Idx);
    Format_String(Output, Line);
  }
}

#include "esp_http_client.h"

static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
  if (new_app_info == NULL)
  {
    return ESP_ERR_INVALID_ARG;
  }

  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_app_desc_t running_app_info;

  if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
  {
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Running firmware version: ");
    Format_String(CONS_UART_Write, running_app_info.version);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
  }
  else
  {
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Failed to get running partition description\n");
    xSemaphoreGive(CONS_Mutex);
    return ESP_FAIL;
  }

  if (memcmp(new_app_info->version, running_app_info.version, sizeof(new_app_info->version)) == 0)
  {
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Current running version is the same as a new. We will not continue the update.\n");
    xSemaphoreGive(CONS_Mutex);
    return ESP_FAIL;
  }

  return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
{
  esp_err_t err = ESP_OK;
  /* Uncomment to add custom headers to HTTP request */
  // err = esp_http_client_set_header(http_client, "Custom-Header", "Value");
  return err;
}

static void DownloadInstallFirwmare(void)
{
  esp_err_t err;
  esp_err_t ota_finish_err = ESP_OK;

  char FirmwareSerialURL[128];
  strcpy(FirmwareSerialURL, Parameters.FirmwareURL);
  strcat(FirmwareSerialURL, ".serial");

  esp_http_client_config_t config = {
      .url = FirmwareSerialURL,
      .timeout_ms = 60000,
      .buffer_size = 4096,
      .keep_alive_enable = true,
  };

  esp_http_client_handle_t Client = esp_http_client_init(&config);

  if (esp_http_client_fetch_headers(Client) < 1)
  {
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA firmware serial headers fetch failed\n");
    xSemaphoreGive(CONS_Mutex);
    esp_http_client_cleanup(Client);
    return;
  }

  char _rx_buffer[32] = {0};
  int read_len = esp_http_client_read(Client, _rx_buffer, sizeof(_rx_buffer) - 1);
  _rx_buffer[sizeof(_rx_buffer) - 1] = '\0';

  esp_https_ota_handle_t https_ota_handle = NULL;
  esp_app_desc_t app_desc = {};
  uint16_t receivedSerialNumber = 0;
  uint16_t nvs_firmwareSerial = 0;

  if (read_len > 0)
  {
    char *endptr = NULL;
    receivedSerialNumber = strtol(_rx_buffer, &endptr, 10);
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA firmware serial received: ");
    Format_UnsDec(CONS_UART_Write, receivedSerialNumber);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
  }
  else
  {
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA firmware serial retrieval failed\n");
    xSemaphoreGive(CONS_Mutex);
  };

  if (receivedSerialNumber == 0)
  {
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA firmware serial retrieval is wrong\n");
    xSemaphoreGive(CONS_Mutex);
    esp_http_client_close(Client);
    esp_http_client_cleanup(Client);
    vTaskDelete(NULL);
  }

  nvs_handle_t _nvsHandle;
  nvs_open("ota", NVS_READONLY, &_nvsHandle);
  nvs_get_u16(_nvsHandle, "serial", &nvs_firmwareSerial);
  nvs_close(_nvsHandle);
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "OTA firmware serial in NVS: ");
  Format_UnsDec(CONS_UART_Write, nvs_firmwareSerial);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);

  if (nvs_firmwareSerial >= receivedSerialNumber)
  {
    esp_http_client_close(Client);
    esp_http_client_cleanup(Client);
    vTaskDelete(NULL);
  }

  // proceed with Firmware download

  config.url = Parameters.FirmwareURL;
  esp_https_ota_config_t ota_config = {
      .http_config = &config,
      .http_client_init_cb = _http_client_init_cb, // Register a callback to be invoked after esp_http_client is initialized
  };

  err = esp_https_ota_begin(&ota_config, &https_ota_handle);
  if (err != ESP_OK)
  {
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "ESP HTTPS OTA Begin failed\n");
    xSemaphoreGive(CONS_Mutex);
    esp_https_ota_abort(https_ota_handle);
    goto ota_end;
  }

  err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
  if (err != ESP_OK)
  {
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "esp_https_ota_get_img_desc failed\n");
    xSemaphoreGive(CONS_Mutex);
    esp_https_ota_abort(https_ota_handle);
    goto ota_end;
  }
  err = validate_image_header(&app_desc);
  if (err != ESP_OK)
  {
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "image header verification failed\n");
    xSemaphoreGive(CONS_Mutex);
    esp_https_ota_abort(https_ota_handle);
    goto ota_end;
  }

  while (1)
  {
    err = esp_https_ota_perform(https_ota_handle);
    if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS)
    {
      break;
    }
    // esp_https_ota_perform returns after every read operation which gives user the ability to
    // monitor the status of OTA upgrade by calling esp_https_ota_get_image_len_read, which gives length of image
    // data read so far.
    const size_t len = esp_https_ota_get_image_len_read(https_ota_handle);
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Image bytes read: ");
    Format_SignDec(CONS_UART_Write, (uint16_t)len);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
  }

  if (esp_https_ota_is_complete_data_received(https_ota_handle) != true)
  {
    // the OTA image was not completely received and user can customise the response to this situation.
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Complete data was not received.\n");
    xSemaphoreGive(CONS_Mutex);
  }
  else
  {
    nvs_open("ota", NVS_READWRITE, &_nvsHandle);
    nvs_firmwareSerial = nvs_set_u16(_nvsHandle, "serial", receivedSerialNumber);
    nvs_commit(_nvsHandle);
    nvs_close(_nvsHandle);

    ota_finish_err = esp_https_ota_finish(https_ota_handle);

    if ((err == ESP_OK) && (ota_finish_err == ESP_OK))
    {
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "ESP_HTTPS_OTA upgrade successful. Rebooting ...\n");
      xSemaphoreGive(CONS_Mutex);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      esp_restart();
    }
    else
    {
      if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED)
      {
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "Image validation failed, image is corrupted\n");
        xSemaphoreGive(CONS_Mutex);
      }
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "ESP_HTTPS_OTA upgrade failed (0x");
      Format_Hex(CONS_UART_Write, (uint8_t)ota_finish_err);
      Format_String(CONS_UART_Write, ")\n");
      xSemaphoreGive(CONS_Mutex);
      esp_https_ota_abort(https_ota_handle);
      goto ota_end;
    }
  }

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "ESP_HTTPS_OTA upgrade failed\n");
  xSemaphoreGive(CONS_Mutex);
ota_end:
  esp_http_client_close(Client);
  esp_http_client_cleanup(Client);
  vTaskDelete(NULL);
}

extern "C"
void vTaskOTA(void *pvParameters)
{
  vTaskDelay(20000);

  for (;;)
  {
    vTaskDelay(60000);
    if (Parameters.FirmwareURL[0] == 0)
      continue;
    if (Flight.inFlight())
    {
      continue;
      // vTaskDelete(NULL);                                          // SP9WPN: left for consideration; if flight detection works well, this might be reasonable
    };
    if (FlashLog_isOpen())
      continue; // don't upload if a log file is being written

    esp_err_t Err = WIFI_Start(); // start WiFi in station-mode
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_Start() => ");
    if (Err >= ESP_ERR_WIFI_BASE)
      Err -= ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif

    vTaskDelay(1000);
    APs = 8;
    Err = WIFI_PassiveScan(AP, APs); // perform a passive scan: find Access Points around
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_PassiveScan() => ");
    if (Err >= ESP_ERR_WIFI_BASE)
      Err -= ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    CONS_UART_Write('/');
    Format_UnsDec(CONS_UART_Write, APs);
    Format_String(CONS_UART_Write, "\n");
    if (Err == ESP_OK)
      AP_Print(CONS_UART_Write);
    xSemaphoreGive(CONS_Mutex);
#endif

    if (Err == ESP_OK) // if WiFi scan went well
    {
      for (uint16_t Idx = 0; Idx < APs; Idx++) // loop over Access Points
      {
        const char *NetName = (const char *)(AP[Idx].ssid);
        const char *NetPass = 0;
        if (AP[Idx].authmode != WIFI_AUTH_OPEN) // if not an open network
        {
          NetPass = Parameters.getWIFIpass(NetName); // then search the password
          if (NetPass == 0)
            continue;
        } // give up if no password for this network
        Err = WIFI_Connect(AP + Idx, NetPass);
#ifdef DEBUG_PRINT
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, NetName);
        if (NetPass)
        {
          CONS_UART_Write('/');
          Format_String(CONS_UART_Write, NetPass);
        }
        CONS_UART_Write(':');
        CONS_UART_Write(' ');
        Format_String(CONS_UART_Write, "WIFI_Connect() => ");
        if (Err >= ESP_ERR_WIFI_BASE)
          Err -= ESP_ERR_WIFI_BASE;
        Format_SignDec(CONS_UART_Write, Err);
        Format_String(CONS_UART_Write, "\n");
        xSemaphoreGive(CONS_Mutex);
#endif
        if (Err)
          continue; // if connection failed then give up ad move on to the next AP

        WIFI_IP.ip.addr = 0;
        WIFI_IP.gw.addr = 0;
        for (uint8_t Idx = 0; Idx < 10; Idx++)
        { // wait to obtain local IP from DHCP
          vTaskDelay(1000);
          // if(WIFI_State.isConnected==1) break;
          if (WIFI_getLocalIP())
          {
            if (WIFI_IP.ip.addr && WIFI_IP.gw.addr)
              break;
          }
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
        if (WIFI_IP.ip.addr == 0)
        {
          WIFI_Disconnect();
          continue;
        } // if getting local IP failed then give up and try another AP

        // here we decide that previous OTA-received firmware works and can be retained
        const esp_partition_t *running = esp_ota_get_running_partition();
        esp_ota_img_states_t ota_state;
        if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
        {
          if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
          {
            if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
            {
              xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
              Format_String(CONS_UART_Write, "App is valid, rollback cancelled successfully\n");
              xSemaphoreGive(CONS_Mutex);
            }
            else
            {
              xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
              Format_String(CONS_UART_Write, "Failed to cancel rollback\n");
              xSemaphoreGive(CONS_Mutex);
            }
          }
        }
        else
        {
          esp_wifi_set_ps(WIFI_PS_NONE);
          DownloadInstallFirwmare();
          WIFI_setPowerSave(1);
        }

        vTaskDelay(2000);
        WIFI_Disconnect();
        WIFI_IP.ip.addr = 0;
        vTaskDelay(2000);
      }
    }

    vTaskDelay(2000);
    Err = WIFI_Stop();
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "WIFI_Stop() => ");
    if (Err >= ESP_ERR_WIFI_BASE)
      Err -= ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
  }
  vTaskDelay(1200000); // wait 20 mins
};

#endif // WITH_WIFI
#endif // WITH_OTA
