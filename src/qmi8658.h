#ifndef __QMI8658_H__
#define __QMI8658_H__

#include <stdint.h>
#include <string.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "hal.h"

class QMI8658
{ private:
   static const uint8_t REG_WHO_AM_I = 0x00;
   static const uint8_t REG_CTRL1    = 0x02;
   static const uint8_t REG_CTRL2    = 0x03;
   static const uint8_t REG_CTRL3    = 0x04;
   static const uint8_t REG_CTRL7    = 0x08;
   static const uint8_t REG_STATUS0  = 0x2E;
   static const uint8_t REG_AX_L     = 0x35;

   static const uint8_t WHO_AM_I     = 0x05;

   static const uint8_t CTRL2_ACC_8G_ODR112HZ     = 0x26; // 8g full scale, 112Hz in 6DOF mode
   static const uint8_t CTRL3_GYRO_2048DPS_ODR112HZ = 0x76; // 2048dps full scale, 112Hz
   static const uint8_t CTRL7_ENABLE_ACC_GYRO     = 0x03;
   static const uint8_t STATUS0_ACC_GYRO_READY    = 0x03;
   static const uint8_t SPI_READ      = 0x80;

  public:
   spi_device_handle_t Device;          // SPI device handle
   uint8_t ID;                          // 0x05 for QMI8658
   uint8_t Error;                       // error on the SPI bus (0=no error)
   uint8_t Status;                      // status register
   int16_t Accel[3];                    // raw acceleration: 4096 counts/g at 8g range
   int16_t Gyro [3];                    // raw gyro: 16 counts/(deg/s) at 2048dps range

  public:
   QMI8658() : Device(0), ID(0), Error(0), Status(0), Accel{0,0,0}, Gyro{0,0,0} { }

   void Close(void)
   { if(Device)
     { spi_bus_remove_device(Device);
       Device=0; } }

   static spi_host_device_t SpiHost(void)
   {
#ifdef SD_SPI_HOST
     return SD_SPI_HOST;
#else
     return SPI3_HOST;
#endif
   }

   uint8_t InitBus(uint8_t Mode=0)
   { pinMode(IMU_PinCS, OUTPUT);
     digitalWrite(IMU_PinCS, HIGH);
#ifdef SD_PinCS
     pinMode(SD_PinCS, OUTPUT);
     digitalWrite(SD_PinCS, HIGH);
#endif
     pinMode(SD_PinMISO, INPUT_PULLUP);
     pinMode(SD_PinMOSI, INPUT_PULLUP);
     pinMode(SD_PinSCK,  INPUT_PULLUP);
     if(Device) return 0;
     spi_device_interface_config_t DevConfig;
     memset(&DevConfig, 0, sizeof(DevConfig));
     DevConfig.clock_speed_hz = 1000000;
     DevConfig.mode = Mode;
     DevConfig.spics_io_num = IMU_PinCS;
     DevConfig.queue_size = 1;
     esp_err_t Ret = spi_bus_add_device(SpiHost(), &DevConfig, &Device);
     if(Ret==ESP_OK) return 0;
     if(Ret!=ESP_ERR_INVALID_STATE) return Ret;
     spi_bus_config_t BusConfig =
     { .mosi_io_num = SD_PinMOSI,
       .miso_io_num = SD_PinMISO,
       .sclk_io_num = SD_PinSCK,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .data4_io_num = -1,
       .data5_io_num = -1,
       .data6_io_num = -1,
       .data7_io_num = -1,
       .max_transfer_sz = 4000,
       .flags = 0,
       .intr_flags = 0 };
#ifdef SD_SPI_DMA
     Ret = spi_bus_initialize(SpiHost(), &BusConfig, SD_SPI_DMA);
#else
     Ret = spi_bus_initialize(SpiHost(), &BusConfig, SPI_DMA_CH_AUTO);
#endif
     if(Ret!=ESP_OK) return Ret;
     Ret = spi_bus_add_device(SpiHost(), &DevConfig, &Device);
     return Ret==ESP_OK ? 0:Ret; }

   uint8_t ReadReg(uint8_t Reg, uint8_t *Data, uint8_t Len)
   { if(Device==0) return ESP_ERR_INVALID_STATE;
     uint8_t Tx[17], Rx[17];
     if(Len>16) return ESP_ERR_INVALID_ARG;
     Tx[0]=Reg|SPI_READ;
     memset(Tx+1, 0, Len);
     memset(Rx, 0, Len+1);
     spi_transaction_t Trans;
     memset(&Trans, 0, sizeof(Trans));
     Trans.length = 8*(Len+1);
     Trans.tx_buffer = Tx;
     Trans.rx_buffer = Rx;
     esp_err_t Ret = spi_device_transmit(Device, &Trans);
     if(Ret!=ESP_OK) return Ret;
     memcpy(Data, Rx+1, Len);
     return 0; }

   uint8_t WriteReg(uint8_t Reg, uint8_t Data)
   { if(Device==0) return ESP_ERR_INVALID_STATE;
     uint8_t Tx[2] = { (uint8_t)(Reg&(~SPI_READ)), Data };
     spi_transaction_t Trans;
     memset(&Trans, 0, sizeof(Trans));
     Trans.length = 16;
     Trans.tx_buffer = Tx;
     esp_err_t Ret = spi_device_transmit(Device, &Trans);
     return Ret==ESP_OK ? 0:Ret; }

   uint8_t CheckID(void)
   { ID=0;
     Error=ReadReg(REG_WHO_AM_I, &ID, 1);
     if(Error) return Error;
     return ID==WHO_AM_I ? 0:0xFF; }

   uint8_t Init(void)
   { Error=InitBus(0); if(Error) return Error;
     Error=CheckID();
     if(Error)
     { Close();
       Error=InitBus(3); if(Error) return Error;
       Error=CheckID(); if(Error) { Close(); return Error; } }
     uint8_t Data=0x60;                                            // internal 2MHz clock, address auto-increment
     Error=WriteReg(REG_CTRL1, Data); if(Error) { Close(); return Error; }
     Data=CTRL2_ACC_8G_ODR112HZ;
     Error=WriteReg(REG_CTRL2, Data); if(Error) { Close(); return Error; }
     Data=CTRL3_GYRO_2048DPS_ODR112HZ;
     Error=WriteReg(REG_CTRL3, Data); if(Error) { Close(); return Error; }
     Data=CTRL7_ENABLE_ACC_GYRO;
     Error=WriteReg(REG_CTRL7, Data);
     if(Error) Close();
     return Error; }

   uint8_t ReadReady(void)
   { Error=ReadReg(REG_STATUS0, &Status, 1);
     if(Error) return Error;
     return (Status&STATUS0_ACC_GYRO_READY)==STATUS0_ACC_GYRO_READY ? 1:0; }

   uint8_t Read(void)
   { uint8_t Data[12];
     Error=ReadReg(REG_AX_L, Data, sizeof(Data));
     if(Error) return Error;
     Accel[0] = (int16_t)(((uint16_t)Data[ 1]<<8) | Data[ 0]);
     Accel[1] = (int16_t)(((uint16_t)Data[ 3]<<8) | Data[ 2]);
     Accel[2] = (int16_t)(((uint16_t)Data[ 5]<<8) | Data[ 4]);
     Gyro [0] = (int16_t)(((uint16_t)Data[ 7]<<8) | Data[ 6]);
     Gyro [1] = (int16_t)(((uint16_t)Data[ 9]<<8) | Data[ 8]);
     Gyro [2] = (int16_t)(((uint16_t)Data[11]<<8) | Data[10]);
     return 0; }
};

#endif // __QMI8658_H__
