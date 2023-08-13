#include <stdint.h>
#include <string.h>
#include <math.h>

#include <Arduino.h>

#include "main.h"
#include "gps.h"
#include "nmea.h"

#include "driver/gpio.h"      // ESP32 GPIO driver
#include "driver/uart.h"      // ESP32 UART driver

#include <Wire.h>             // I2C for pressure sensor and charge controller
#include <SPI.h>

#include "nvs.h"
#include "nvs_flash.h"

// =======================================================================================================

uint64_t getUniqueMAC(void)                                  // 48-bit unique ID of the ESP32 chip
{ uint8_t MAC[6]; esp_efuse_mac_get_default(MAC);
  uint64_t ID=MAC[0];
  for(int Idx=1; Idx<6; Idx++)
  { ID<<=8; ID|=MAC[Idx]; }
  return ID; }

uint64_t getUniqueID(void) { return getUniqueMAC(); }         // get unique serial ID of the CPU/chip
uint32_t getUniqueAddress(void) { return getUniqueMAC()&0x00FFFFFF; } // get unique OGN address

// =======================================================================================================

static int NVS_Init(void)
{ esp_err_t Err = nvs_flash_init();
  if (Err == ESP_ERR_NVS_NO_FREE_PAGES)
  { nvs_flash_erase();
    Err = nvs_flash_init(); }
  return Err; }

// =======================================================================================================

FlashParameters Parameters;  // parameters stored in Flash: address, aircraft type, etc.

// =======================================================================================================
// CONSole UART

void CONS_UART_Write(char Byte) // write byte to the console (USB serial port)
{ Serial.write(Byte); }

int  CONS_UART_Free(void)
{ return Serial.availableForWrite(); }

int  CONS_UART_Read (uint8_t &Byte)
{ int Ret=Serial.read(); if(Ret<0) return 0;
  Byte=Ret; return 1; }

// =======================================================================================================

#define GPS_UART    UART_NUM_1      // UART for GPS

const int GPS_PinTx  = GPIO_NUM_12;
const int GPS_PinRx  = GPIO_NUM_34;
const int GPS_PinPPS = GPIO_NUM_37;

int   GPS_UART_Full         (void)          { size_t Full=0; uart_get_buffered_data_len(GPS_UART, &Full); return Full; }
int   GPS_UART_Read         (uint8_t &Byte) { return uart_read_bytes  (GPS_UART, &Byte, 1, 0); }  // should be buffered and non-blocking
void  GPS_UART_Write        (char     Byte) {        uart_write_bytes (GPS_UART, &Byte, 1);    }  // should be buffered and blocking
void  GPS_UART_Flush        (int MaxWait  ) {        uart_wait_tx_done(GPS_UART, MaxWait);     }
void  GPS_UART_SetBaudrate  (int BaudRate ) {        uart_set_baudrate(GPS_UART, BaudRate);    }

bool GPS_PPS_isOn(void) { return gpio_get_level((gpio_num_t)GPS_PinPPS); }

static void GPS_UART_Init(int BaudRate=9600)
{ gpio_set_direction((gpio_num_t)GPS_PinPPS, GPIO_MODE_INPUT);

  uart_config_t GPS_UART_Config =
  { baud_rate : BaudRate,
    data_bits : UART_DATA_8_BITS,
    parity    : UART_PARITY_DISABLE,
    stop_bits : UART_STOP_BITS_1,
    flow_ctrl : UART_HW_FLOWCTRL_DISABLE,
    rx_flow_ctrl_thresh: 0,
    use_ref_tick: 0
  };
  uart_param_config  (GPS_UART, &GPS_UART_Config);
  uart_set_pin       (GPS_UART, GPS_PinTx, GPS_PinRx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  uart_driver_install(GPS_UART, 256, 256, 0, 0, 0);
  uart_set_rx_full_threshold(GPS_UART, 16); }

// =======================================================================================================

void LED_PCB_On   (void) { }                // LED on the PCB for visual indications
void LED_PCB_Off  (void) { }
void LED_PCB_Flash(uint8_t Time) { }

// =======================================================================================================

uint8_t PowerMode = 2;                    // 0=sleep/minimal power, 1=comprimize, 2=full power

SemaphoreHandle_t CONS_Mutex;                // Mut-Ex for the Console
SemaphoreHandle_t I2C_Mutex;                 // Mut-Ex for the I2C

void setup()
{
  CONS_Mutex = xSemaphoreCreateMutex();      // semaphore for sharing the writing to the console
  I2C_Mutex  = xSemaphoreCreateMutex();      // semaphore for sharing the I2C bus

  NVS_Init();                                // initialize storage in flash like for parameters

  Parameters.setDefault(getUniqueAddress()); // set default parameter values
  if(Parameters.ReadFromNVS()!=ESP_OK)       // try to get parameters from NVS
  { Parameters.WriteToNVS(); }               // if did not work: try to save (default) parameters to NVS
  if(Parameters.CONbaud<2400 || Parameters.CONbaud>921600 || Parameters.CONbaud%2400)
  { Parameters.CONbaud=115200; Parameters.WriteToNVS(); }

  Serial.begin(Parameters.CONbaud);          // USB Console: baud rate probably does not matter here
  GPS_UART_Init();

  xTaskCreate(vTaskGPS    ,  "GPS"  ,  4000, NULL, 1, NULL);  // task to read data from GPS

}

// =======================================================================================================

void PrintTasks(void (*CONS_UART_Write)(char))
{ char Line[32];

  size_t FreeHeap = xPortGetFreeHeapSize();
  Format_String(CONS_UART_Write, "Task            Pr. Stack, ");
  Format_UnsDec(CONS_UART_Write, (uint32_t)FreeHeap, 4, 3);
  Format_String(CONS_UART_Write, "kB free\n");

  UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
  TaskStatus_t *pxTaskStatusArray = (TaskStatus_t *)pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );
  if(pxTaskStatusArray==0) return;
  uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, NULL );
  for(UBaseType_t T=0; T<uxArraySize; T++)
  { TaskStatus_t *Task = pxTaskStatusArray+T;
    uint8_t Len=Format_String(Line, Task->pcTaskName, configMAX_TASK_NAME_LEN, 0);
    // for( ; Len<=configMAX_TASK_NAME_LEN; )
    //   Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, Task->uxCurrentPriority, 2); Line[Len++]=' ';
    // Line[Len++]='0'+Task->uxCurrentPriority; Line[Len++]=' ';
    Len+=Format_UnsDec(Line+Len, (uint32_t)(Task->usStackHighWaterMark), 3);
    Line[Len++]='\n'; Line[Len]=0;
    Format_String(CONS_UART_Write, Line);
  }
  vPortFree( pxTaskStatusArray );
}

static NMEA_RxMsg NMEA;
static char Line[128];

static void PrintParameters(void)                               // print parameters stored in Flash
{ Parameters.Print(Line);
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);                   // ask exclusivity on UART1
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex);                                  // give back UART1 to other tasks
}

static void PrintPOGNS(void)                                   // print parameters in the $POGNS form
{ xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Parameters.WritePOGNS(Line);
  Format_String(CONS_UART_Write, Line);
  Parameters.WritePOGNS_Pilot(Line);
  Format_String(CONS_UART_Write, Line);
  Parameters.WritePOGNS_Acft(Line);
  Format_String(CONS_UART_Write, Line);
  Parameters.WritePOGNS_Comp(Line);
  Format_String(CONS_UART_Write, Line);
#ifdef WITH_AP
  Parameters.WritePOGNS_AP(Line);
  Format_String(CONS_UART_Write, Line);
#endif
#ifdef WITH_STRATUX
  Parameters.WritePOGNS_Stratux(Line);
  Format_String(CONS_UART_Write, Line);
#endif
  xSemaphoreGive(CONS_Mutex);                                          //
  return; }

#ifdef WITH_CONFIG
static void ReadParameters(void)  // read parameters requested by the user in the NMEA sent.
{ if((!NMEA.hasCheck()) || NMEA.isChecked() )
  { PrintParameters();
    if(NMEA.Parms==0) { PrintPOGNS(); return; }                              // if no parameter given
    Parameters.ReadPOGNS(NMEA);
    PrintParameters();
    esp_err_t Err = Parameters.WriteToNVS();                                                  // erase and write the parameters into the Flash
  }
}
#endif

static void ProcessNMEA(void)     // process a valid NMEA that got to the console
{
#ifdef WITH_CONFIG
  if(NMEA.isPOGNS()) ReadParameters();
#endif
}

static void ProcessCtrlC(void)                                  // print system state to the console
{ xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Parameters.Print(Line);
  // PrintTasks(CONS_UART_Write);                               // print the FreeRTOS tasks
  Parameters.Write(CONS_UART_Write);                         // write the parameters to the console
  xSemaphoreGive(CONS_Mutex); }

static int ProcessInput(void)
{ int Count=0;
  for( ; ; )
  { uint8_t Byte; int Err=CONS_UART_Read(Byte); if(Err<=0) break; // get byte from console, if none: exit the loop
    Count++;
    if(Byte==0x03) ProcessCtrlC();                                // if Ctrl-C received: print parameters
    if(Byte==0x18) esp_restart();                                 // Ctrl-X

    NMEA.ProcessByte(Byte);                                       // pass the byte through the NMEA processor
    if(NMEA.isComplete())                                         // if complete NMEA:
    { ProcessNMEA();                                              // interpret the NMEA
      NMEA.Clear(); }                                             // clear the NMEA processor for the next sentence
  }
  return Count; }

void loop()
{ if(ProcessInput()==0) vTaskDelay(1);

}

// =======================================================================================================
