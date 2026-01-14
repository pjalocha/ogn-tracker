#ifdef WITH_OTA_HTTPS
#ifdef WITH_WIFI

#include "main.h"

#include "socket.h"
#include "wifi.h"
#include "log.h"
#include "gps.h"

#include "ssl_root_ca.h"
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

extern "C" bool verifyRollbackLater()
{
  return true;
}

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

void url_strip_to_path(char *url)
{
  char *last_slash;

  if (url == NULL)
    return;

  /* Remove query string or fragment if present */
  url[strcspn(url, "?#")] = '\0';

  /* Find the last slash */
  last_slash = strrchr(url, '/');

  if (last_slash == NULL)
    return;

  /* If the slash is not the final character, truncate after it */
  if (*(last_slash + 1) != '\0')
    *(last_slash + 1) = '\0';
}

static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
  if (new_app_info == NULL)
  {

#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Error reading new firmware header. Aborting update.\n");
    xSemaphoreGive(CONS_Mutex);
#endif

    return ESP_ERR_INVALID_ARG;
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

static void DownloadTrackerSettings(void)
{
  //
  // Download and write parameters for specific tracker
  // filename is "settings_a1b2c3.txt"
  //    a1b1c3 is tracker ID (lowercase)
  // file should be located in FirmwareURL directory
  // file syntax is Name=Value
  //

  esp_err_t err;

#define SETTINGS_FILE_MAX_SIZE 4096
  char *SettingsURL = (char *)malloc(160);

  strcpy(SettingsURL, Parameters.FirmwareURL);

  url_strip_to_path(SettingsURL);
  char _setFileName[30];
  sprintf(_setFileName, "settings_%06x.txt", Parameters.Address);
  strcat(SettingsURL, _setFileName);

  esp_http_client_config_t config = {
      .url = SettingsURL,
      .cert_pem = CACERTPEM,
      .timeout_ms = 10000,
      .keep_alive_enable = false,
  };

  esp_http_client_handle_t Client = esp_http_client_init(&config);
  err = esp_http_client_open(Client, 0);

  char *buffer = (char *)malloc(SETTINGS_FILE_MAX_SIZE + 1);
  int content_length = esp_http_client_fetch_headers(Client);

  if (content_length <= 0)
  {
    free(buffer);
    free(SettingsURL);
    esp_http_client_close(Client);
    esp_http_client_cleanup(Client);
    return;
  }

  int total_read = 0;
  while (total_read < SETTINGS_FILE_MAX_SIZE)
  {
    int r = esp_http_client_read(Client, buffer + total_read, SETTINGS_FILE_MAX_SIZE - total_read);

    if (r < 0)
    {
      buffer[0] = '\0'; // ERROR
    }
    if (r == 0)
    {
      break; // EOF
    }

    total_read += r;
  }

  buffer[total_read] = '\0';

  /* Split into lines */
  char *saveptr;
  char *line = strtok_r(buffer, "\n", &saveptr);

  while (line)
  {
    /* Strip trailing CR if present */
    size_t len = strlen(line);
    if (len > 0 && line[len - 1] == '\r')
    {
      line[len - 1] = '\0';
    }

    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, line);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);

    Parameters.ReadLine(line);
    line = strtok_r(NULL, "\n", &saveptr);
  }

  Parameters.WriteToNVS();
  free(buffer);
  free(SettingsURL);
  esp_http_client_close(Client);
  esp_http_client_cleanup(Client);
}

static void DownloadInstallFirmware(void)
{
  esp_err_t err;
  esp_err_t ota_finish_err = ESP_OK;
  nvs_handle_t _nvsHandle;

  char _rx_buffer[32] = {0};

  //
  // Check firmware serial on server
  //
  // file is same as firmware, but with ".serial" added
  // should contain an u32 number incremented with each new serial
  // suggested format is: 2025102701 <- date YMD and daily serial number

  char *FirmwareSerialURL = (char *)malloc(160);
  // char FirmwareSerialURL[160] = {0};
  strcpy(FirmwareSerialURL, Parameters.FirmwareURL);
  strcat(FirmwareSerialURL, ".serial");

  esp_http_client_config_t config = {
      .url = FirmwareSerialURL,
      .cert_pem = CACERTPEM,
      .timeout_ms = 60000,
      .buffer_size = 8192,
      .keep_alive_enable = true,
  };

  esp_http_client_handle_t Client = esp_http_client_init(&config);

  if (esp_http_client_open(Client, 0) != ESP_OK)
  {
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA http connection open failed\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    free(FirmwareSerialURL);
    esp_http_client_close(Client);
    esp_http_client_cleanup(Client);
    return;
  }

  if (esp_http_client_fetch_headers(Client) < 1)
  {
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA firmware serial headers fetch failed\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    free(FirmwareSerialURL);
    esp_http_client_close(Client);
    esp_http_client_cleanup(Client);
    return;
  }
  vTaskDelay(20); // give other tasks a chance

  int read_len = esp_http_client_read(Client, _rx_buffer, sizeof(_rx_buffer) - 1);
  _rx_buffer[sizeof(_rx_buffer) - 1] = '\0';

  free(FirmwareSerialURL);
  esp_http_client_close(Client);
  esp_http_client_cleanup(Client);

  esp_https_ota_handle_t https_ota_handle = NULL;
  esp_app_desc_t app_desc = {};
  uint32_t receivedSerialNumber = 0;
  uint32_t nvs_firmwareSerial = 0;
  if (read_len > 0)
  {
    char *endptr = NULL;
    receivedSerialNumber = strtol(_rx_buffer, &endptr, 10);
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA firmware serial received: ");
    Format_UnsDec(CONS_UART_Write, receivedSerialNumber);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
  }
  else
  {
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA firmware serial retrieval failed\n");
    xSemaphoreGive(CONS_Mutex);
#endif
  };
  if (receivedSerialNumber == 0)
  {
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA firmware serial retrieval went wrong\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    xSemaphoreGive(WIFI_Mutex);
    vTaskDelete(NULL);
  }
  vTaskDelay(50); // give other tasks a chance

  nvs_open("ota", NVS_READONLY, &_nvsHandle);
  nvs_get_u32(_nvsHandle, "serial", &nvs_firmwareSerial);
  nvs_close(_nvsHandle);

#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "OTA firmware serial in NVS: ");
  Format_UnsDec(CONS_UART_Write, nvs_firmwareSerial);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif

  if (nvs_firmwareSerial >= receivedSerialNumber)
  {
    xSemaphoreGive(WIFI_Mutex);
    vTaskDelete(NULL);
    return;
  }

  //
  // proceed with Firmware download
  //

  vTaskPrioritySet(NULL, 10);         // increase even more

  config.url = Parameters.FirmwareURL;
  config.buffer_size = 8192;
  config.timeout_ms = 60000;

  esp_https_ota_config_t ota_config = {
      .http_config = &config,
      .http_client_init_cb = _http_client_init_cb, // Register a callback to be invoked after esp_http_client is initialized
  };

  err = esp_https_ota_begin(&ota_config, &https_ota_handle);
  if (err != ESP_OK)
  {
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "ESP HTTPS OTA Begin failed\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    esp_https_ota_abort(https_ota_handle);
    xSemaphoreGive(WIFI_Mutex);
    return;
  }

  vTaskDelay(20); // give other tasks a chance

  err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
  if (err != ESP_OK)
  {
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "esp_https_ota_get_img_desc failed\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    esp_https_ota_abort(https_ota_handle);
    xSemaphoreGive(WIFI_Mutex);
    vTaskDelete(NULL);
    return;
  }
  err = validate_image_header(&app_desc);
  if (err != ESP_OK)
  {
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "image header verification failed\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    esp_https_ota_abort(https_ota_handle);
    xSemaphoreGive(WIFI_Mutex);
    vTaskDelete(NULL);
    return;
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
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA bytes read: ");
    Format_UnsDec(CONS_UART_Write, (uint32_t)len);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif
    vTaskDelay(20); // give other tasks a chance
  }

  if (esp_https_ota_is_complete_data_received(https_ota_handle) != true)
  {
// the OTA image was not completely received and user can customise the response to this situation.
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Complete data was not received.\n");
    xSemaphoreGive(CONS_Mutex);
#endif
  }
  else
  {
    ota_finish_err = esp_https_ota_finish(https_ota_handle);
    if ((err == ESP_OK) && (ota_finish_err == ESP_OK))
    {
      // write candidate serial number to NVS
      // if OTA fails and firmware is rolled back, this will prevent from downloading same failed again
      nvs_open("ota", NVS_READWRITE, &_nvsHandle);
      nvs_set_u32(_nvsHandle, "serial", receivedSerialNumber);
      nvs_commit(_nvsHandle);
      nvs_close(_nvsHandle);

      Parameters.WriteToNVS();

#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "ESP_HTTPS_OTA upgrade successful. Rebooting in 3 seconds...\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      vTaskDelay(3000);
      esp_restart();
    }
    else
    {
      if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED)
      {
#ifdef DEBUG_PRINT
        xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
        Format_String(CONS_UART_Write, "Image validation failed, image is corrupted\n");
        xSemaphoreGive(CONS_Mutex);
#endif
      }
#ifdef DEBUG_PRINT
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "ESP_HTTPS_OTA upgrade failed (0x");
      Format_Hex(CONS_UART_Write, (uint8_t)ota_finish_err);
      Format_String(CONS_UART_Write, ")\n");
      xSemaphoreGive(CONS_Mutex);
#endif
      esp_https_ota_abort(https_ota_handle);
      esp_http_client_close(Client);
      esp_http_client_cleanup(Client);
      xSemaphoreGive(WIFI_Mutex);
      vTaskDelete(NULL);
      return;
    }
  }

#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "ESP_HTTPS_OTA upgrade failed\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  esp_http_client_close(Client);
  esp_http_client_cleanup(Client);

  xSemaphoreGive(WIFI_Mutex);
  vTaskDelete(NULL);
}

extern "C" void vTaskOTA(void *pvParameters)
{
  vTaskDelay(15000);
  for (;;)
  {
    if (Flight.inFlight())
    {
      vTaskDelete(NULL);
    };

    if (Parameters.FirmwareURL[0] == 0)
    {
      vTaskDelay(30000);
      continue;
    }

    xSemaphoreTake(WIFI_Mutex, portMAX_DELAY);
    vTaskDelay(3000);

    esp_err_t Err = WIFI_Start(); // start WiFi in station-mode
#ifdef DEBUG_PRINT
    if (Err >= ESP_ERR_WIFI_BASE)
    {
      Err -= ESP_ERR_WIFI_BASE;
      xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
      Format_String(CONS_UART_Write, "OTA WIFI_Start() error => ");
      Format_SignDec(CONS_UART_Write, Err);
      Format_String(CONS_UART_Write, "\n");
      xSemaphoreGive(CONS_Mutex);
    }
#endif

    vTaskDelay(1000);
    APs = 8;
    Err = WIFI_PassiveScan(AP, APs); // perform a passive scan: find Access Points around
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA WIFI_PassiveScan() => ");
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
        Format_String(CONS_UART_Write, "OTA WIFI_Connect() => ");
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
        if (WIFI_IP.ip.addr == 0) // if getting local IP failed then give up and try another AP
        {
          WIFI_Disconnect();
          continue;
        }

        // here we decide that previous OTA-received firmware works and can be retained
        // our test condition is successful WIFI connection
        // this also implies ca. 20 seconds of tracker uptime (vTask delays)
        const esp_partition_t *running = esp_ota_get_running_partition();
        esp_ota_img_states_t ota_state;
        if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
        {
          if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
          {
            if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
            {
              // NOTE: serial number in NVS is already bumped to prevent redownloading failed firmwares
              nvs_handle_t _nvsHandle;
              uint32_t NVSserial;
              nvs_open("ota", NVS_READONLY, &_nvsHandle);
              nvs_get_u32(_nvsHandle, "serial", &NVSserial);
              nvs_close(_nvsHandle);

              // write new serial number to "Soft" parameter so it gets broadcasted
              char _soft[16] = {0};
              sprintf(_soft, "ota%d", (NVSserial));
              _soft[15] = '\0';
              strcpy(Parameters.Soft, _soft);
              Parameters.WriteToNVS();

#ifdef DEBUG_PRINT
              xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
              Format_String(CONS_UART_Write, "New APP firmware is valid, rollback cancelled.\n");
              xSemaphoreGive(CONS_Mutex);
#endif
            }
            else
            {
#ifdef DEBUG_PRINT
              xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
              Format_String(CONS_UART_Write, "Failed to cancel rollback\n");
              xSemaphoreGive(CONS_Mutex);
#endif
            }
          }
          else
          {
            vTaskPrioritySet(NULL, 5);
            esp_wifi_set_ps(WIFI_PS_NONE); // disable WIFI powersaving
            DownloadTrackerSettings();
            DownloadInstallFirmware();
            vTaskPrioritySet(NULL, 0);
            WIFI_setPowerSave(1);
          }
        }
        vTaskDelay(1000);
        WIFI_Disconnect();
        WIFI_IP.ip.addr = 0;
        vTaskDelay(1000);
      }
    }

    vTaskDelay(1000);
    Err = WIFI_Stop();
#ifdef DEBUG_PRINT
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "OTA WIFI_Stop() => ");
    if (Err >= ESP_ERR_WIFI_BASE)
      Err -= ESP_ERR_WIFI_BASE;
    Format_SignDec(CONS_UART_Write, Err);
    Format_String(CONS_UART_Write, "\n");
    xSemaphoreGive(CONS_Mutex);
#endif

    xSemaphoreGive(WIFI_Mutex);
    vTaskDelay(300000); // wait 5 mins
  }
};

void print_ota_status()
{
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "--- Partition Status ---\n");
  xSemaphoreGive(CONS_Mutex);

  // Find all app partitions
  esp_partition_iterator_t iterator = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);

  if (iterator == NULL)
  {
    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "No app partitions found.\n");
    xSemaphoreGive(CONS_Mutex);
    return;
  }

  while (iterator != NULL)
  {
    const esp_partition_t *partition = esp_partition_get(iterator);
    esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;

    // Get the state of the partition
    // Note: esp_ota_get_state_partition returns ESP_OK if successful, but we need to check the 'ota_state' variable
    esp_ota_get_state_partition(partition, &ota_state);

    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    Format_String(CONS_UART_Write, "Name: ");
    Format_String(CONS_UART_Write, partition->label);
    Format_String(CONS_UART_Write, " SubType: ");
    Format_Hex(CONS_UART_Write, (uint8_t)partition->subtype);
    Format_String(CONS_UART_Write, " Size: ");
    Format_Hex(CONS_UART_Write, (uint32_t)partition->size);
    Format_String(CONS_UART_Write, " State: ");
    xSemaphoreGive(CONS_Mutex);

    xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
    // Map the state enum to a readable string
    switch (ota_state)
    {
    case ESP_OTA_IMG_NEW:
      Format_String(CONS_UART_Write, "NEW\n");
      break;
    case ESP_OTA_IMG_PENDING_VERIFY:
      Format_String(CONS_UART_Write, "PENDING_VERIFY\n");
      break;
    case ESP_OTA_IMG_VALID:
      Format_String(CONS_UART_Write, "VALID\n");
      break;
    case ESP_OTA_IMG_INVALID:
      Format_String(CONS_UART_Write, "INVALID\n");
      break;
    case ESP_OTA_IMG_ABORTED:
      Format_String(CONS_UART_Write, "ABORTED\n");
      break;
    case ESP_OTA_IMG_UNDEFINED:
    default:
      Format_String(CONS_UART_Write, "UNDEFINED: ");
      Format_UnsDec(CONS_UART_Write, ota_state);
      Format_String(CONS_UART_Write, "\n");
      break;
    }
    xSemaphoreGive(CONS_Mutex);

    // Move to the next partition
    iterator = esp_partition_next(iterator);
  }

  // Release the iterator
  esp_partition_iterator_release(iterator);

  // Print currently running and next boot partitions
  const esp_partition_t *running_partition = esp_ota_get_running_partition();
  const esp_partition_t *next_partition = esp_ota_get_next_update_partition(running_partition);

  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "Running Partition: ");
  Format_String(CONS_UART_Write, running_partition->label);
  Format_String(CONS_UART_Write, "\n");
  if (next_partition)
  {
    Format_String(CONS_UART_Write, "Next Boot Partition (if not marked valid): ");
    Format_String(CONS_UART_Write, next_partition->label);
    Format_String(CONS_UART_Write, "\n");
  }
  Format_String(CONS_UART_Write, "------------------------\n");
  xSemaphoreGive(CONS_Mutex);
}

#endif // WITH_WIFI
#endif // WITH_OTA
