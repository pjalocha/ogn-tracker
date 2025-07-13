#include <stdint.h>
#include <string.h>
#include <math.h>

#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <Arduino.h>

#include <Wire.h>             // I2C for pressure sensor and charge controller
#include <SPI.h>

#include "main.h"

#include "esp_core_dump.h"

#ifdef WITH_BT4_SPP
#include "bt4.h"
#endif

#ifdef WITH_BT_SPP
#include "BluetoothSerial.h"
#endif

#ifdef WITH_BLE_SPP
#include "ble_spp.h"
#endif

#ifdef WITH_WIFI
#include "wifi.h"
#endif

#ifdef WITH_UPLOAD
#include "upload.h"
#endif

#ifdef WITH_AP
#include "ap.h"
#endif

#ifdef WITH_XPOWERS
#include "XPowersLib.h"
#endif

#ifdef WITH_AXP
#include <axp20x.h>
#endif

#ifdef Button_Pin
#include <esp_sleep.h>
#include "Button2.h"
#endif

// #include "driver/temperature_sensor.h"  // internal ESP32 temperature sensor

#include "oled.h"

#include "gps.h"
// #include "rf.h"
#include "ogn-radio.h"
#include "sens.h"
#include "proc.h"
#include "log.h"

#include "nmea.h"

#include "driver/gpio.h"      // ESP32 GPIO driver
#include "driver/uart.h"      // ESP32 UART driver

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_vfs_fat.h"
#include "esp_spiffs.h"

#ifdef WITH_ST7735
#include "tft.h"
#endif

// =======================================================================================================

uint64_t getUniqueMAC(void)                                  // 48-bit unique ID of the ESP32 chip
{ uint8_t MAC[6]; esp_efuse_mac_get_default(MAC);
  uint64_t ID=MAC[0];
  for(int Idx=1; Idx<6; Idx++)
  { ID<<=8; ID|=MAC[Idx]; }
  return ID; }

uint64_t getUniqueID(void) { return getUniqueMAC(); }         // get unique serial ID of the CPU/chip
uint32_t getUniqueAddress(void) { return getUniqueMAC()&0x00FFFFFF; } // get unique OGN address

Word32x2 Random = { 0x0123456789ABCDEF };

HardItems HardwareStatus = { 0 };

// =======================================================================================================
// VS to store parameters and other data

static int NVS_Init(void)
{ esp_err_t Err = nvs_flash_init();
  if (Err == ESP_ERR_NVS_NO_FREE_PAGES)
  { nvs_flash_erase();
    Err = nvs_flash_init(); }
  return Err; }

// =======================================================================================================

#ifdef WITH_SPIFFS

#ifdef WITH_SPIFFS_FAT // FAT replaces SPIFFS, hopefully no performace and reliability issues

int SPIFFS_Register(const char *Path, const char *Label, size_t MaxOpenFiles)
{ esp_vfs_fat_mount_config_t FSconf;
  FSconf.max_files = MaxOpenFiles;
  FSconf.format_if_mount_failed = true;
  FSconf.allocation_unit_size = 4096;
  static wl_handle_t Handle = WL_INVALID_HANDLE;
  return esp_vfs_fat_spiflash_mount(Path, Label, &FSconf, &Handle); }

int SPIFFS_Info(size_t &Total, size_t &Used, const char *Label)
{ FATFS *FS=0;
  Total=0; Used=0;
  size_t FreeClusters;
  int Ret = f_getfree("0:", &FreeClusters, &FS);
  // if(Ret=!FR_OK) return Ret;
  if(FS==0) return 0;
  size_t TotalSectors = (FS->n_fatent-2) * FS->csize;
  size_t FreeSectors = FreeClusters * FS->csize;
  Total = TotalSectors * FS->ssize;
  Used  = (TotalSectors-FreeSectors) * FS->ssize;
  return 0; }

#else // SPIFFS: gives troubles when more than few files are open

int SPIFFS_Register(const char *Path, const char *Label, size_t MaxOpenFiles)
{ esp_vfs_spiffs_conf_t FSconf =
  { base_path: Path,
    partition_label: Label,
    max_files: MaxOpenFiles,
    format_if_mount_failed: true };
  return esp_vfs_spiffs_register(&FSconf); }

int SPIFFS_Info(size_t &Total, size_t &Used, const char *Label)
{ return esp_spiffs_info(Label, &Total, &Used); }

#endif
#endif

// =======================================================================================================

uint8_t I2C_Restart(uint8_t Bus)
{ Wire.end(); Wire.begin(); return 0; }

uint8_t I2C_Read (uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait)
{ Wire.beginTransmission(Addr);
  int Ret=Wire.write(Reg);
  Wire.endTransmission(false);
  Ret=Wire.requestFrom(Addr, Len);
  for(uint8_t Idx=0; Idx<Len; Idx++)
  { Data[Idx]=Wire.read(); }
  return Ret!=Len; }

uint8_t I2C_Write(uint8_t Bus, uint8_t Addr, uint8_t Reg, uint8_t *Data, uint8_t Len, uint8_t Wait)
{ Wire.beginTransmission(Addr);
  int Ret=Wire.write(Reg);
  for(uint8_t Idx=0; Idx<Len; Idx++)
  { Ret=Wire.write(Data[Idx]); if(Ret!=1) break; }
  Wire.endTransmission();
  return Ret!=1; }

// =======================================================================================================

#ifdef WITH_XPOWERS
XPowersLibInterface *PMU = 0;
#endif

#ifdef WITH_AXP
static AXP20X_Class AXP;
#endif

uint8_t PowerMode = 2;                       // 0=sleep/minimal power, 1=comprimize, 2=full power

#ifdef Vext_PinEna
static void Vext_Init(void) {  pinMode(Vext_PinEna, OUTPUT); }
static void Vext_ON(bool ON=1) { digitalWrite(Vext_PinEna, ON); }
#endif

// =======================================================================================================

#ifdef WITH_ST7735

const  uint8_t  TFT_Pages      = 6;       // six LCD pages
static uint8_t  TFT_Page       = 0;       // page currently on display
static uint8_t  TFT_PageChange = 0;       // signal the page has been changed
static uint8_t  TFT_PageOFF    = 0;       // Backlight to be OFF
#ifdef WITH_TFT_DIM
static uint32_t TFT_PageActive = 0;       // [ms] last time the page was active (button pressed)
const  uint32_t TFT_PageTimeout = (uint32_t)60000*WITH_TFT_DIM;  // [ms] timeout to turn off the TFT backlight
#endif

static void TFT_NextPage(void)
{ TFT_Page++;
  if(TFT_Page>=TFT_Pages) TFT_Page=0;
  TFT_PageChange=1; }

static int TFT_DrawPage(const GPS_Position *GPS)
{ // Serial.printf("TFT_DrawPage() TFT_Page:%d TFT_PageChange:%d\n", TFT_Page, TFT_PageChange);
  if(TFT_Page==1) return TFT_DrawID();
  if(TFT_Page==2) return TFT_DrawSat();
  if(TFT_Page==3) return TFT_DrawRF();
  if(!GPS) return TFT_DrawID();
  if(TFT_Page==4) return TFT_DrawBaro(GPS);
  if(TFT_Page==5) return TFT_DrawLoRaWAN(GPS);
  return TFT_DrawGPS(GPS);
  return 0; }

#endif

// =======================================================================================================

#ifdef Button_Pin
static Button2 Button(Button_Pin);
static bool Button_isPressed(void) { return digitalRead(Button_Pin)==0; }

static void Button_Single(Button2 Butt)
{
#ifdef WITH_ST7735
  if(TFT_PageOFF)
    TFT_PageOFF=0;
  else
    TFT_NextPage();
  TFT_PageActive=millis();
#endif
}

static void Button_Double(Button2 Butt) { }

static void Button_Long(Button2 Butt)
{
#ifdef WITH_ST7735
  TFT.fillScreen(ST77XX_DARKBLUE);
  TFT.setTextColor(ST77XX_WHITE);
  TFT.setFont(0);
  TFT.setTextSize(2);
  TFT.setCursor(32, 32);
  TFT.print("Power-OFF");
  delay(200);
  TFT_BL(64);
  delay(50);
  TFT_BL(32);
  delay(50);
  TFT_BL(16);
  delay(50);
#endif
#ifdef WITH_SLEEP
  Parameters.PowerON=0;
  Parameters.WriteToNVS();
  PowerMode=0;
  Vext_ON(0);
  esp_deep_sleep_start();
#endif
}

static void Button_Init(void)
{ pinMode(Button_Pin, INPUT);
  Button.setLongClickTime(2000);
  Button.setClickHandler(Button_Single);
  Button.setDoubleClickHandler(Button_Double);
  Button.setLongClickDetectedHandler(Button_Long); }
#endif

// =======================================================================================================
// ADC to sense battery voltage

static esp_adc_cal_characteristics_t *ADC_characs =
        (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
#ifdef BATT_ADC_CHANNEL
static const adc1_channel_t ADC_Chan_Batt = BATT_ADC_CHANNEL;  // ADC1_CHANNEL_0 for GPIO 1 or ADC1_CHANNEL_3 for GPIO 4
#else
static const adc1_channel_t ADC_Chan_Batt = ADC1_CHANNEL_7; // ADC channel #7 is GPIO35
#endif
static const adc_atten_t ADC_atten = ADC_ATTEN_DB_11;
static const adc_unit_t ADC_unit = ADC_UNIT_1;
#define ADC_Vref 1100

static int ADC_Init(void)
{ // if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) // Check TP is burned into eFuse
  // if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) // Check Vref is burned into eFuse
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_Chan_Batt, ADC_atten);
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_unit, ADC_atten, ADC_WIDTH_BIT_12, ADC_Vref, ADC_characs); // calibrate ADC1
#ifdef ADC_BattSenseEna
  pinMode(ADC_BattSenseEna, OUTPUT);
  digitalWrite(ADC_BattSenseEna, LOW);
#endif
  return 0; }

#ifdef ADC_BattSenseEna
static void BatterySenseEnable(bool ON=1) { digitalWrite(ADC_BattSenseEna, ON); }
#endif

uint16_t BatterySense(int Samples)  // [mV] read battery voltage from power-control chip or from an ADC channel
{
#ifdef WITH_XPOWERS
  if(PMU) return PMU->getBattVoltage();
#endif
#ifdef WITH_AXP
  if(HardwareStatus.AXP192 || HardwareStatus.AXP202) return AXP.getBattVoltage();
#endif
#ifdef ADC_BattSenseEna
  digitalWrite(ADC_BattSenseEna, HIGH);
  delay(1);
#endif
  uint32_t RawVoltage=0;
  for( int Idx=0; Idx<Samples; Idx++)
  { RawVoltage += adc1_get_raw(ADC_Chan_Batt); }
  RawVoltage = (RawVoltage)/Samples;

  uint16_t Volt = (uint16_t)esp_adc_cal_raw_to_voltage(RawVoltage, ADC_characs);
#ifdef BATT_ADC_RATIO
  Volt = Volt*BATT_ADC_RATIO;
#else
  Volt = Volt*2;
#endif

#ifdef BATT_ADC_BIAS
  if(Volt>=BATT_ADC_BIAS) Volt-=BATT_ADC_BIAS;
#else
  const uint16_t Bias = 50;  // apparently, there is 80mV bias in the battery voltage measurement
  if(Volt>=Bias) Volt-=Bias;
#endif
#ifdef ADC_BattSenseEna
  digitalWrite(ADC_BattSenseEna, LOW);
#endif
  return Volt; } // [mV]

// =======================================================================================================

FlashParameters Parameters;  // parameters stored in Flash: address, aircraft type, etc.

// =======================================================================================================
// CONSole UART

#ifdef WITH_BT_SPP
static BluetoothSerial BTserial;
#endif

void CONS_UART_Write(char Byte) // write byte to the console (USB serial port)
{ Serial.write(Byte);
#ifdef WITH_BT4_SPP
  BT_SPP_Write(Byte);
#endif
#ifdef WITH_BT_SPP
  BTserial.write(Byte);
#endif
#ifdef WITH_BLE_SPP
  BLE_SPP_TxFIFO.Write(Byte);
#endif
}

int  CONS_UART_Free(void)
{ return Serial.availableForWrite(); }

int  CONS_UART_Read (uint8_t &Byte)
{ char Char;
  int Ret=Serial.read(); if(Ret>=0) { Byte=Ret; return 1; }
#ifdef WITH_BT4_SPP
  Ret=BT_SPP_Read(Byte); if(Ret>0) { return 1; }
#endif
#ifdef WITH_BLE_SPP
  Ret=BLE_SPP_RxFIFO.Read(Char); if(Ret>0) { Byte=Char; return 1; }
#endif
#ifdef WITH_BT_SPP
  Ret=BTserial.read(); if(Ret>=0) { Byte=Ret; return 1; }
#endif
  return 0; }

// =======================================================================================================

#ifdef GPS_PinPPS
uint32_t PPS_Intr_msTime = 0;   // [ms] xTaskGetTickCount() counter at the time of the PPS
uint32_t PPS_Intr_usTime = 0;   // [us] micros() counter at the time of the PPS

uint32_t PPS_usPrecTime  = 0;   // [1/16us] precise time of the PPS
uint32_t PPS_usTimeRMS   = 0;   // [1/4us]  precise PPS time residue time error RMS

uint32_t PPS_Intr_usFirst= 0;   // [us] micros() counter at the first interrupt in a series
uint32_t PPS_Intr_Count  = 0;   // [count] good PPS interrupts in the series
uint32_t PPS_Intr_Missed = 0;   // [count] missed PPS interrupts

 int32_t PPS_usPeriodErr = 0;   // [1/16us] PPS period average systematic error
uint32_t PPS_usPeriodRMS = 0;   // [(1/4us)^2] mean square of statistical error

const uint32_t PPS_usPeriod = 1000000;  // [usec] expected PPS period = 1sec = 10000000usec

static void IRAM_ATTR PPS_Intr(void *Context)
{ uint32_t usTime = micros();                                     // [usec] usec-clock at interrupt time
  uint32_t msTime = xTaskGetTickCount();                          // [msec] mses-clock at interrupt time
  uint32_t usDelta = usTime - PPS_Intr_usTime;                    // difference from the previous PPS
  PPS_Intr_usTime = usTime;                                       // [usec]
  PPS_Intr_msTime = msTime;                                       // [ms]
  uint32_t Cycles = (usDelta+PPS_usPeriod/2)/PPS_usPeriod;        // how many PPS periods past
   int32_t usResid = usDelta-Cycles*PPS_usPeriod;                 // [usec]
  if(Cycles>0 && Cycles<=10 && abs(usResid)<Cycles*500)           // condition to accept the PPS edge
  { int32_t ResidErr = (usResid<<4)-PPS_usPeriodErr;              // [1/16us]
    if(Cycles>1) ResidErr/=Cycles;
    PPS_usPeriodErr += (ResidErr+2)>>2;                         // [1/16us] average the PPS period error
    uint32_t ErrSqr = ResidErr*ResidErr;
    if(PPS_Intr_Count)                                            // if not the first edge in the series
    { PPS_usPrecTime += ((PPS_usPeriod<<4)+PPS_usPeriodErr)*Cycles;  // [1/16us] forcast the PPS time to the current PPS
      PPS_usPeriodRMS += ((int32_t)((ErrSqr>>4)-PPS_usPeriodRMS)+2)>>2;
      int32_t usTimeErr = (usTime<<4)-PPS_usPrecTime;                // [1/16us] difference between current PPs and the forecast
      PPS_usPrecTime += (usTimeErr+2)>>2;                            // [1/16us]
      uint32_t ErrSqr = usTimeErr*usTimeErr;
      PPS_usTimeRMS += ((int32_t)((ErrSqr>>4)-PPS_usTimeRMS)+2)>>2; }
    else                                                          // if this was the very first edge in the series
    { PPS_usPrecTime  = PPS_Intr_usTime<<4;
      PPS_usTimeRMS   = 4<<4;
      PPS_usPeriodErr = ResidErr;
      PPS_usPeriodRMS = 4<<4; }                                   // set statistical error RMS to 2 usec
    PPS_Intr_Count += Cycles;
    PPS_Intr_Missed+= Cycles-1; }                                 // count PPS cycles
  else                                                            // if edge is not accepted
  { PPS_Intr_Count=0;                                             // then we start all from scratch
    PPS_Intr_Missed=0;
    PPS_Intr_usFirst= usTime; }
}
#endif

// move to the specific pin-defnition file
// #define GPS_UART    UART_NUM_1      // UART for GPS
// const int GPS_PinTx  = GPIO_NUM_12;
// const int GPS_PinRx  = GPIO_NUM_34;
// const int GPS_PinPPS = GPIO_NUM_37;

int   GPS_UART_Full         (void)          { size_t Full=0; uart_get_buffered_data_len(GPS_UART, &Full); return Full; }
int   GPS_UART_Read         (uint8_t &Byte) { return uart_read_bytes  (GPS_UART, &Byte, 1, 0); }  // should be buffered and non-blocking
int   GPS_UART_Read(uint8_t *Data, int Max) { return uart_read_bytes  (GPS_UART, Data, Max, 0); }
void  GPS_UART_Write        (char     Byte) {        uart_write_bytes (GPS_UART, &Byte, 1);    }  // should be buffered and blocking
void  GPS_UART_Flush        (int MaxWait  ) {        uart_wait_tx_done(GPS_UART, MaxWait);     }
void  GPS_UART_SetBaudrate  (int BaudRate ) {        uart_set_baudrate(GPS_UART, BaudRate);    }

#ifdef GPS_PinPPS
bool GPS_PPS_isOn(void) { return gpio_get_level((gpio_num_t)GPS_PinPPS); }
#endif
#ifdef GPS_PinEna
void GPS_DISABLE(void) { gpio_set_level((gpio_num_t)GPS_PinEna, 0); }
void GPS_ENABLE (void) { gpio_set_level((gpio_num_t)GPS_PinEna, 1); }
#endif

static void GPS_UART_Init(int BaudRate=9600)
{
#ifdef GPS_PinPPS
  // gpio_set_direction((gpio_num_t)GPS_PinPPS, GPIO_MODE_INPUT);
  gpio_config_t PinConf = { };
  PinConf.intr_type    = GPIO_INTR_POSEDGE;    // Interrupt on rising edge
  PinConf.mode         = GPIO_MODE_INPUT;      // Set as input
  PinConf.pin_bit_mask = (1ULL << GPS_PinPPS); // Select pin
  PinConf.pull_down_en = (gpio_pulldown_t)0;   // Disable pull-down
  PinConf.pull_up_en   = (gpio_pullup_t)0;     // Disable pull-up
  gpio_config(&PinConf);
  gpio_install_isr_service(0);                 // default flags, but what could it be ?
  gpio_isr_handler_add((gpio_num_t)GPS_PinPPS, PPS_Intr, 0);   // the last arg. is context which can be passed to the interrupt handl$#endif
#endif
#ifdef GPS_PinEna
  gpio_set_direction((gpio_num_t)GPS_PinEna, GPIO_MODE_OUTPUT);
  GPS_ENABLE();
#endif
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

  uart_driver_install(GPS_UART, 512, 512, 0, 0, 0);
  uart_set_rx_full_threshold(GPS_UART, 32); }

// =======================================================================================================

#ifdef LED_PCB_Pin
static void LED_PCB_Init(void)  { pinMode(LED_PCB_Pin, OUTPUT); }
       void LED_PCB_On(bool ON) { digitalWrite(LED_PCB_Pin, ON); }
       void LED_PCB_Off(void)   { return LED_PCB_On(0); }
       void LED_PCB_Flash(uint8_t Time) { LED_PCB_On(1); delay(1); LED_PCB_On(0); }
#else
static void LED_PCB_Init (void)    { }
       void LED_PCB_On   (bool ON) { }
       void LED_PCB_Off  (void)    { }
       void LED_PCB_Flash(uint8_t Time) { }
#endif

// =======================================================================================================

SemaphoreHandle_t CONS_Mutex;                // Mut-Ex for the Console
SemaphoreHandle_t I2C_Mutex;                 // Mut-Ex for the I2C

static char Line[128];
static void PrintPOGNS(void);

void setup()
{
  CONS_Mutex = xSemaphoreCreateMutex();      // semaphore for sharing the writing to the console
  I2C_Mutex  = xSemaphoreCreateMutex();      // semaphore for sharing the I2C bus

  NVS_Init();                                // initialize storage in flash like for parameters
#ifdef WITH_SPIFFS
  SPIFFS_Register();                         // initialize the file system in the Flash
#endif

// #ifdef WITH_OGN
//   Radio_SlotMsg = xQueueCreate(1, sizeof(TimeSync)); // message queue for GPS to signal the new time slot
// #endif
  Random.Word+=getUniqueID(); XorShift64(Random.Word);

#ifdef Button_Pin
  Button_Init();
#endif
  LED_PCB_Init();
#ifdef WITH_HTIT_TRACKER
  Vext_Init();
  Vext_ON();
#endif
  Parameters.setDefault(getUniqueAddress()); // set default parameter values
  if(Parameters.ReadFromNVS()!=ESP_OK)       // try to get parameters from NVS
  { Parameters.WriteToNVS(); }               // if did not work: try to save (default) parameters to NVS
  if(Parameters.CONbaud<2400 || Parameters.CONbaud>921600 || Parameters.CONbaud%2400)
  { Parameters.CONbaud=115200; Parameters.WriteToNVS(); }

#ifdef HARD_NAME
  strcpy(Parameters.Hard, HARD_NAME);
#endif
#ifdef SOFT_NAME
  strcpy(Parameters.Soft, SOFT_NAME);
#endif

#ifdef ARDUINO_USB_MODE
  Serial.setTxTimeoutMs(0);                  // to prevent delays and blocking of threads which send data to the USB console
#endif
  Serial.begin(Parameters.CONbaud);          // USB Console: baud rate probably does not matter here
  GPS_UART_Init();

  Serial.printf("OGN-Tracker: Hard:%s Soft:%s\n", Parameters.Hard, Parameters.Soft);

#ifdef BOARD_HAS_PSRAM
  if(psramFound()) Serial.printf("PSRAM:%d/%dkB ", ESP.getFreePsram()>>10, ESP.getPsramSize()>>10);
#endif
  Serial.printf("Heap:%d/%dkB CPU:%dMHz\n", ESP.getFreeHeap()>>10, ESP.getHeapSize()>>10, getCpuFrequencyMhz());

#ifdef WITH_ST7735
  TFT_Init();
  TFT.setRotation(1);
  TFT.fillScreen(ST77XX_DARKBLUE);
  TFT_BL_Init();
  TFT_BL(0);
#ifdef WITH_SLEEP
  if(!Parameters.PowerON)                  // if the tracker has been turned off by the user
  { TFT.setTextColor(ST77XX_WHITE);
    TFT.setFont(0);
    TFT.setTextSize(2);
    TFT.setCursor(32, 16);
    TFT.print("Confirm");                  // ask for confirmation to avoid acidential turn on
    TFT.setCursor(30, 48);
    TFT.print("Power-ON");
    uint8_t BLlev=0;
    int Pressed=0;
    for(int Wait=0; Wait<2000; Wait++)     // wait 2sec for confirmation
    { delay(1);
      if(BLlev<128) { BLlev++; TFT_BL(BLlev); }
      if(!Button_isPressed()) { Pressed=0; continue; }
      Pressed++;
      if(Pressed>=20) { Parameters.PowerON=1; break; } // if button pressed for 20ms
    }
    if(Parameters.PowerON)                 // if the button pressed
    { Parameters.WriteToNVS(); }
    else                                   // if not pressed: user did not confirm power-on
    { TFT_BL(0);                           // backlight to zero
      Vext_ON(0);                          // turn off external devices
#ifdef ADC_BattSenseEna
      BatterySenseEnable(0);
#endif
      esp_deep_sleep_start(); }            // enter deep sleep
  }
#endif  // WITH_SLEEP
  TFT_BL(128);
#endif  // WITH_ST7735

#ifdef I2C_PinSCL
  Wire.begin(I2C_PinSDA, I2C_PinSCL, (uint32_t)400000); // (SDA, SCL, Frequency) I2C on the correct pins
  Wire.setTimeOut(20);                                  // [ms]
#endif
#ifdef PMU_I2C_PinSCL
  static TwoWire PMU_I2C = TwoWire(1);
  PMU_I2C.begin(PMU_I2C_PinSDA, PMU_I2C_PinSCL, (uint32_t)400000);
  PMU_I2C.setTimeOut(20);
#endif

#ifdef WITH_AXP
#ifdef PMU_I2C_PinSCL
  if(AXP.begin(PMU_I2C, AXP192_SLAVE_ADDRESS)!=AXP_FAIL)
#else
  if(AXP.begin(Wire, AXP192_SLAVE_ADDRESS)!=AXP_FAIL)
#endif
  { HardwareStatus.AXP192=1; Serial.println("Power/charge chip AXP192 detected"); }
#ifdef PMU_I2C_PinSCL
  else if(AXP.begin(PMU_I2C, AXP202_SLAVE_ADDRESS)!=AXP_FAIL)
#else
  else if(AXP.begin(Wire, AXP202_SLAVE_ADDRESS)!=AXP_FAIL)
#endif
  { HardwareStatus.AXP202=1; Serial.println("Power/charge chip AX202 detected"); }
  else
  { Serial.println("AXP power/charge chip NOT detected"); }

  if(HardwareStatus.AXP192 || HardwareStatus.AXP202)
  { AXP.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                   AXP202_VBUS_CUR_ADC1 |
                   AXP202_BATT_CUR_ADC1 |
                   AXP202_BATT_VOL_ADC1,
                   true);
    Serial.printf("  USB:  %5.3fV  %5.3fA\n",
              0.001f*AXP.getVbusVoltage(), 0.001f*AXP.getVbusCurrent());
    Serial.printf("  Batt: %5.3fV (%5.3f-%5.3f)A\n",
              0.001f*AXP.getBattVoltage(), 0.001f*AXP.getBattChargeCurrent(), 0.001f*AXP.getBattDischargeCurrent());
  }
#endif
#ifdef WITH_XPOWERS
  if(PMU==0)
  {
#ifdef PMU_I2C_PinSCL
    PMU = new XPowersAXP2101(PMU_I2C);
#else
    PMU = new XPowersAXP2101(Wire);
#endif
    if(PMU->init())
    { HardwareStatus.AXP210=1; Serial.println("Power/charge chip AXP2101 detected"); }
    else
    { delete PMU; PMU=0; Serial.println("Power/charge chip AXP2101 NOT detected"); }
  }
  if(PMU==0)
  {
#ifdef PMU_I2C_PinSCL
    PMU = new XPowersAXP192(PMU_I2C);
#else
    PMU = new XPowersAXP192(Wire);
#endif
    if(PMU->init())
    { HardwareStatus.AXP192=1; Serial.println("Power/charge chip AXP192 detected"); }
    else
    { delete PMU; PMU=0; Serial.println("Power/charge chip AXP192 NOT detected"); }
  }
  if(HardwareStatus.AXP210)
  { PMU->enableSystemVoltageMeasure();
    PMU->enableVbusVoltageMeasure();
    PMU->enableBattVoltageMeasure();
    // It is necessary to disable the detection function of the TS pin on the board
    // without the battery temperature detection function, otherwise it will cause abnormal charging
    PMU->disableTSPinMeasure();
#ifdef WITH_TBEAM12
    // RF 3300mV
    PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO2);
    // GPS 3300mV
    PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO3);
#endif
#ifdef WITH_TBEAMS3
    // GNSS RTC PowerVDD 3300mV
    // PMU->setPowerChannelVoltage(XPOWERS_VBACKUP, 3300);
    // PMU->enablePowerOutput(XPOWERS_VBACKUP);
#ifdef WITH_BME280
    PMU->setPowerChannelVoltage(XPOWERS_ALDO1, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO1);
#endif
#ifdef WITH_OLED
    PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO2);
#endif
    // RF VDD 3300mV
    PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO3);
    // GNSS VDD 3300mV
    PMU->setPowerChannelVoltage(XPOWERS_ALDO4, 3300);
    PMU->enablePowerOutput(XPOWERS_ALDO4);
#endif
    // set charging LED flashing
    PMU->setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ); }
  if(HardwareStatus.AXP192 || HardwareStatus.AXP210)
  { Serial.printf("  USB:  %5.3fV\n", 0.001f*PMU->getVbusVoltage());
    Serial.printf("  Batt: %5.3fV %d%%\n", 0.001f*PMU->getBattVoltage(), PMU->getBatteryPercent());
    // Serial.printf("  USB:  %5.3fV  %5.3fA\n",
    //           0.001f*PMU->getVbusVoltage(), 0.001f*PMU->getVbusCurrent());
    // Serial.printf("  Batt: %5.3fV (%5.3f-%5.3f)A\n",
    //           0.001f*PMU->getBattVoltage(), 0.001f*PMU->getBattChargeCurrent(), 0.001f*PMU->getBattDischargeCurrent());
  }
#endif
  if(!HardwareStatus.AXP192 && !HardwareStatus.AXP202 && !HardwareStatus.AXP210)  // if none of the power controllers detected
  { ADC_Init(); }                                               // then we use ADC to measue the battery voltage

/*
  Serial.printf("I2C scan:");
  uint8_t I2Cdev=0;
  for(uint8_t Addr=0x01; Addr<128; Addr++)
  { Wire.beginTransmission(Addr);
    if(Wire.endTransmission(Addr)==0) { Serial.printf(" 0x%02X", Addr); I2Cdev++; }
                                 else { Wire.flush(); }
  }
  Serial.printf(" %d devices\n", I2Cdev);
*/

  char Info[512];
  int InfoLen=0;
  esp_core_dump_init();
  esp_core_dump_summary_t *DumpSummary = (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
  if(DumpSummary)
  { esp_err_t Err = esp_core_dump_get_summary(DumpSummary);
    if (Err==ESP_OK)
    { esp_core_dump_bt_info_t &BackTrace = DumpSummary->exc_bt_info;
      InfoLen=sprintf(Info, "Crash-dump: Task:%s PC:%08x [%d]\n", DumpSummary->exc_task, DumpSummary->exc_pc, BackTrace.depth);
      for(uint32_t Idx=0; Idx<BackTrace.depth && Idx<16; Idx++)
      { InfoLen+=sprintf(Info+InfoLen, " 0x%08x", BackTrace.bt[Idx]); }
    }
    free(DumpSummary); }
  if(InfoLen) Serial.println(Info);

#ifdef WITH_AP                    // with WiFi Access Point
#ifdef WITH_AP_BUTTON
    bool StartAP = Button_isPressed() && Parameters.APname[0]; // start WiFi AP when button pressed during startup and APname non-empty
#else
    bool StartAP = Parameters.APname[0]; // start WiFi AP when APname non-empty
#endif
#else  // WITH_AP
    const bool StartAP=0;
#endif // WITH_AP

#ifdef WITH_WIFI
  WIFI_State.Flags=0;
  esp_err_t Err=WIFI_Init();
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "WIFI_Init() => ");
  if(Err>=ESP_ERR_WIFI_BASE) Err-=ESP_ERR_WIFI_BASE;
  Format_SignDec(CONS_UART_Write, Err);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
#endif

#ifdef WITH_BT4_SPP
  if(!StartAP && Parameters.BTname[0])
  { int32_t Err=BT_SPP_Init();
    Serial.printf("Start BT4 (ESP-IDF) Serial Port: %s (%d)\n", Parameters.BTname, Err); }
#endif

#ifdef WITH_BT_SPP
  if(!StartAP && Parameters.BTname[0])
  { Serial.printf("Start BT4 (Arduino) Serial Port: %s\n", Parameters.BTname);
    BTserial.begin(Parameters.BTname); }
#endif

#ifdef WITH_BLE_SPP
  if(!StartAP && Parameters.BTname[0])
  { Serial.printf("Start BLE (Arduino) Serial Port: %s\n", Parameters.BTname);
    BLE_SPP_Start(Parameters.BTname); }
#endif

#ifdef WITH_OLED
  OLED.begin();
  OLED.setDisplayRotation(U8G2_R2);
  OLED.clearBuffer();
  OLED_DrawLogo(OLED.getU8g2(), 0);
  OLED.sendBuffer();
#endif
#ifdef WITH_ST7735
  TFT_DrawID(StartAP);
#endif

  uint8_t Len=Format_String(Line, "$POGNS,SysStart");
  Len+=NMEA_AppendCheckCRNL(Line, Len);
  Line[Len]=0;
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, Line);
  xSemaphoreGive(CONS_Mutex);
  PrintPOGNS();

#ifdef WITH_LOG
  xTaskCreate(vTaskLOG    ,  "LOG"  ,  5000, NULL, 0, NULL);  // log data to flash
#endif
  xTaskCreate(vTaskGPS    ,  "GPS"  ,  4000, NULL, 1, NULL);  // read data from GPS
#if defined(WITH_BMP180) || defined(WITH_BMP280) || defined(WITH_BME280)
  xTaskCreate(vTaskSENS   ,  "SENS" ,  4000, NULL, 1, NULL);  // read data from pressure sensor
#endif
  xTaskCreate(vTaskPROC   ,  "PROC" ,  4000, NULL, 0, NULL);  // process received packets, prepare packets for transmission
  xTaskCreate(Radio_Task  ,  "RF"   ,  4000, NULL, 1, NULL);  // transmit/receive packets
#ifdef WITH_AP
  if(StartAP)
    xTaskCreate(vTaskAP,  "AP",  4000, NULL, 0, NULL);
#endif
#ifdef WITH_UPLOAD
  xTaskCreate(vTaskUPLOAD,"UPLOAD",4000, NULL, 0, NULL);
#endif

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
#ifdef WITH_GPS_UBX_PASS
static UBX_RxMsg  UBX;
#endif

static void PrintParameters(void)                              // print parameters stored in Flash
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

#ifdef WITH_LOG
static void ListLogFile(void)
{ if(NMEA.Parms!=1) return;
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "ListLogFile() ");
  Format_String(CONS_UART_Write, (const char *)NMEA.ParmPtr(0), 0, 12);
  Format_String(CONS_UART_Write, " ");
  Format_UnsDec(CONS_UART_Write, (uint32_t)NMEA.ParmLen(0));
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  uint32_t FileTime = FlashLog_ReadShortFileTime((const char *)NMEA.ParmPtr(0), NMEA.ParmLen(0));
  if(FileTime==0) return;
#ifdef DEBUG_PRINT
  xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Format_String(CONS_UART_Write, "ListLogFile() ");
  Format_Hex(CONS_UART_Write, FileTime);
  Format_String(CONS_UART_Write, "\n");
  xSemaphoreGive(CONS_Mutex);
#endif
  FlashLog_ListFile(FileTime);
}
#endif

static void ProcessNMEA(void)     // process a valid NMEA that we got to the console
{
#ifdef WITH_CONFIG
  if(NMEA.isPOGNS()) ReadParameters();
#endif
#ifdef WITH_LOG
  if(NMEA.isPOGNL()) ListLogFile();
#endif
}

#ifdef WITH_LORAWAN
static void PrintLoRaWAN()
{ if(!xSemaphoreTake(CONS_Mutex, 100)) return;
  bool Joined = (WANdev.State>>1)&1;
  const char *State = Joined?"Conn.":"not-Conn.";
  if(!WANdev.Enable) State = "Disabled";
  Serial.printf("LoRaWAN: %s", State);
  if(Joined) Serial.printf(" %08X:%08X", WANdev.HomeNetID, WANdev.DevAddr);
  Serial.printf(" Nonce:%08X Up:%u Dn:%d Tx:%u Rx:%u %ddBm %+4.1fdB",
           WANdev.DevNonce, WANdev.UpCount, WANdev.DnCount, WANdev.TxCount, WANdev.RxCount,
           WANdev.RxRSSI, 0.25*WANdev.RxSNR);
  Serial.printf("\n");
  xSemaphoreGive(CONS_Mutex); }
#endif

static void ProcessCtrlF(void)                                  // list log files to the console
{ xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
#ifdef WITH_SPIFFS
  char FullName[32];
  strcpy(FullName, "/spiffs/");
  struct stat Stat;
  uint32_t Files=0;                                          // count/list files in SPIFFS
  DIR *Dir=opendir(FullName);                                // open SPIFFS top directory
  if(Dir)
  { for( ; ; )                                               // loop over files
    { struct dirent *Ent = readdir(Dir); if(!Ent) break;     // get the next file of the directory
      if(Ent->d_type != DT_REG) continue;                    // if not a regular file (directory, link, ...): then skip
      char *Name = Ent->d_name;
      strcpy(FullName+8, Name);
      if(stat(FullName, &Stat)<0) continue;
      Format_String(CONS_UART_Write, FullName);
      CONS_UART_Write(' ');
      Format_UnsDec(CONS_UART_Write, (uint32_t)Stat.st_size);
      // if(Stat.st_size==0) { unlink(FullName); }           // remove files with zero length
      Format_String(CONS_UART_Write, "\n");
      Files++; }                                             // count the (regular) files
    closedir(Dir); }
  Format_String(CONS_UART_Write, "SPIFFS: ");
  size_t Total, Used;
  if(SPIFFS_Info(Total, Used)==0)                            // get the SPIFFS usage summary
  { Format_UnsDec(CONS_UART_Write, Used/1024);
    Format_String(CONS_UART_Write, "kB used, ");
    Format_UnsDec(CONS_UART_Write, Total/1024);
    Format_String(CONS_UART_Write, "kB total, "); }
  Format_UnsDec(CONS_UART_Write, Files);
  Format_String(CONS_UART_Write, " files\n");
#endif
  xSemaphoreGive(CONS_Mutex); }

static void ProcessCtrlC(void)                                  // print system state to the console
{ xSemaphoreTake(CONS_Mutex, portMAX_DELAY);
  Parameters.Print(Line);
  Format_String(CONS_UART_Write, Line);
  Format_String(CONS_UART_Write, "GPS: ");
  Format_UnsDec(CONS_UART_Write, GPS_getBaudRate(), 1);
  Format_String(CONS_UART_Write, "bps");
  CONS_UART_Write(',');
  Format_UnsDec(CONS_UART_Write, GPS_PosPeriod, 4, 3);
  CONS_UART_Write('s');
  if(GPS_Status.PPS)         Format_String(CONS_UART_Write, ",PPS");
  if(GPS_Status.NMEA)        Format_String(CONS_UART_Write, ",NMEA");
  if(GPS_Status.UBX)         Format_String(CONS_UART_Write, ",UBX");
  if(GPS_Status.MAV)         Format_String(CONS_UART_Write, ",MAV");
  if(GPS_Status.BaudConfig)  Format_String(CONS_UART_Write, ",BaudOK");
  if(GPS_Status.ModeConfig)  Format_String(CONS_UART_Write, ",ModeOK");
  CONS_UART_Write('\r'); CONS_UART_Write('\n');
  // PrintTasks(CONS_UART_Write);                               // print the FreeRTOS tasks

  Parameters.Write(CONS_UART_Write);                         // write the parameters to the console

  Format_String(CONS_UART_Write, "Batt:");
  Format_UnsDec(CONS_UART_Write, (10*BatteryVoltage+128)>>8, 5, 4);
  Format_String(CONS_UART_Write, "V ");
  Format_SignDec(CONS_UART_Write, (600*BatteryVoltageRate+128)>>8, 3, 1);
  Format_String(CONS_UART_Write, "mV/min\n");

  xSemaphoreGive(CONS_Mutex); }

static void ProcessCtrlX(void)
{ static uint32_t LastTime=0;
  uint32_t Time=millis();
  uint32_t Diff=Time-LastTime;
  if(Diff<1000)
  { // SysLog_Line("Restart from console", 1, 50);
    // ShutDownReq=1;
#ifdef WITH_SPIFFS
    FlashLog_SaveReq=1;
#endif
    vTaskDelay(2000);
    ESP.restart(); }
  LastTime=Time; }

static void ProcessCtrlL(void)                                    // print system state to the console
{
#ifdef WITH_SPIFFS
  FlashLog_ListFiles();
#endif
}

static void ProcessCtrlO(void)                                    // print system state to the console
{
#ifdef WITH_LORAWAN
  PrintLoRaWAN();
#endif
}

static int ProcessInput(void)
{
  const uint8_t CtrlB = 'B'-'@';
  const uint8_t CtrlC = 'C'-'@';
  const uint8_t CtrlF = 'F'-'@';
  const uint8_t CtrlL = 'L'-'@';
  const uint8_t CtrlO = 'O'-'@';
  const uint8_t CtrlX = 'X'-'@';

  int Count=0;
  for( ; ; )
  { uint8_t Byte; int Err=CONS_UART_Read(Byte); if(Err<=0) break; // get byte from console, if none: exit the loop
    Count++;
#ifndef WITH_GPS_UBX_PASS                                          // when transparency to the GPS not requested
    if(Byte==CtrlC) ProcessCtrlC();                                // if Ctrl-C received: print parameters
    if(Byte==CtrlF) ProcessCtrlF();                                // if Ctrl-F received: list files
    if(Byte==CtrlL) ProcessCtrlL();                                // if Ctrl-L received: list log files
    if(Byte==CtrlO) ProcessCtrlO();                                // if Ctrl-O received: print LoRaWAN status
    if(Byte==CtrlX) ProcessCtrlX();                                // Ctrl-X
//     {
// #ifdef WITH_SPIFFS
//       FlashLog_SaveReq=1;
// #endif
//       vTaskDelay(1000);
//       esp_restart(); }                                            // if Ctrl-X received then restart
#endif // of WITH_GPS_UBX_PASS

    NMEA.ProcessByte(Byte);                                       // pass the byte through the NMEA processor
    if(NMEA.isComplete())                                         // if complete NMEA:
    {
#ifdef WITH_GPS_NMEA_PASS
      if(NMEA.isChecked())
        NMEA.Send(GPS_UART_Write);
#endif
      ProcessNMEA();                                              // interpret the NMEA
      NMEA.Clear(); }                                             // clear the NMEA processor for the next sentence
#ifdef WITH_GPS_UBX_PASS
    UBX.ProcessByte(Byte);
    if(UBX.isComplete())
    { UBX.Send(GPS_UART_Write);                                   // is there a need for a Mutex on the GPS UART ?
      UBX.Clear(); }
#endif
  }
  return Count; }

void loop()
{ vTaskDelay(1);
#ifdef Button_Pin
  Button.loop();
#endif
#ifdef WITH_BLE_SPP
  BLE_SPP_Check();
#endif
  // if(ProcessInput()==0) vTaskDelay(1);
  while(ProcessInput()>0);
#ifdef WITH_ST7735
  static GPS_Position *PrevGPS=0;
  GPS_Position *GPS = GPS_getPosition();
  if(GPS==0) { GPS = GPS_Pos+GPS_PosIdx; }
  // if(GPS && !GPS->isTimeValid()) GPS==0;
  if(TFT_PageChange)
  { TFT_PageChange=0;
    if(TFT_DrawPage(GPS)==0) TFT_NextPage(); }
  if(GPS!=PrevGPS)
  { TFT_PageChange=1;
#ifdef WITH_TFT_DIM
    uint32_t msTime = millis();
    if(BatteryVoltageRate>0x10) TFT_PageActive = msTime;
    uint32_t Age = msTime-TFT_PageActive;
    TFT_PageOFF = Age>TFT_PageTimeout;
#else
    TFT_PageOFF = 0;
#endif
    if(TFT_PageOFF) TFT_BL(0);
              else  TFT_BL(128);
    PrevGPS=GPS; }
#endif
}

// =======================================================================================================
