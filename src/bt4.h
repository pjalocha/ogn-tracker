#ifndef __BT4_H__
#define __BT4_H__

#include <stdint.h>

int  BT_SPP_Init(void);
bool BT_SPP_isConnected(void);
 int BT_SPP_Read (uint8_t &Byte);
void BT_SPP_Write (char Byte);

#endif // __BT4_H__
