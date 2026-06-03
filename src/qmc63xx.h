#ifndef __QMC63XX_H__
#define __QMC63XX_H__

#include <stdint.h>

#include "hal.h"

class QMC63XX
{ private:
   static const uint8_t ADDR_QMC6310A = 0x1C;
   static const uint8_t ADDR_QMC6310B = 0x3C;
   static const uint8_t ADDR_QMC6309  = 0x7C;

   static const uint8_t REG_ID        = 0x00;
   static const uint8_t REG_DATA      = 0x01;
   static const uint8_t REG_STATUS    = 0x09;
   static const uint8_t REG_CTRL1     = 0x0A;
   static const uint8_t REG_CTRL2     = 0x0B;
   static const uint8_t STATUS_DRDY   = 0x01;
   static const uint8_t CTRL2_RNG_2G  = 0x0C; // QMC6310: 15000 LSB/G = 150 LSB/uT

  public:
   enum Type_t
   { Type_None    = 0,
     Type_QMC6310 = 1,
     Type_QMC6309 = 2 };

   uint8_t Bus;                         // which I2C bus
   uint8_t ADDR;                        // detected I2C address
   uint8_t ID;                          // chip ID: 0x80 = QMC6310, 0x90 = QMC6309
   Type_t  Type;
   uint8_t Error;                       // error on the I2C bus (0=no error)
   uint8_t Status;                      // status register
   int16_t Mag[3];                      // raw magnetic field readout

  public:
   QMC63XX() : Bus(0), ADDR(0), ID(0), Type(Type_None), Error(0), Status(0), Mag{0,0,0} { }

   const char *Name(void) const
   { if(Type==Type_QMC6310) return "QMC6310";
     if(Type==Type_QMC6309) return "QMC6309";
     return "none"; }

   uint8_t CheckID(void)
   { static const uint8_t Addr[3] = { ADDR_QMC6310A, ADDR_QMC6310B, ADDR_QMC6309 };
     ADDR=0; ID=0; Type=Type_None;
     for(uint8_t Idx=0; Idx<3; Idx++)
     { uint8_t ChipID=0;
       Error=I2C_Read(Bus, Addr[Idx], REG_ID, ChipID);
       if(Error) continue;
       if(ChipID==0x80)
       { ADDR=Addr[Idx]; ID=ChipID; Type=Type_QMC6310; return 0; }
       if(ChipID==0x90)
       { ADDR=Addr[Idx]; ID=ChipID; Type=Type_QMC6309; return 0; }
     }
     return Error ? Error:0xFF; }

   uint8_t Init(void)
   { Error=CheckID(); if(Error) return Error;
     uint8_t Data=0x00;
     Error=I2C_Write(Bus, ADDR, REG_CTRL1, Data); if(Error) return Error; // suspend before changing mode
     if(Type==Type_QMC6310)
     { Data=CTRL2_RNG_2G;
       Error=I2C_Write(Bus, ADDR, REG_CTRL2, Data); if(Error) return Error;
       Data=0x07; }                                                       // 50Hz continuous mode
     else
     { Data=0x00;
       Error=I2C_Write(Bus, ADDR, REG_CTRL2, Data); if(Error) return Error;
       Data=0x03; }                                                       // continuous mode
     Error=I2C_Write(Bus, ADDR, REG_CTRL1, Data);
     return Error; }

   uint8_t ReadReady(void)
   { Error=I2C_Read(Bus, ADDR, REG_STATUS, Status);
     if(Error) return Error;
     return (Status&STATUS_DRDY) ? 1:0; }

   uint8_t Read(void)
   { uint8_t Data[6];
     Error=I2C_Read(Bus, ADDR, REG_DATA, Data, sizeof(Data));
     if(Error) return Error;
     Mag[0] = (int16_t)(((uint16_t)Data[1]<<8) | Data[0]);
     Mag[1] = (int16_t)(((uint16_t)Data[3]<<8) | Data[2]);
     Mag[2] = (int16_t)(((uint16_t)Data[5]<<8) | Data[4]);
     return 0; }
};

#endif // __QMC63XX_H__
