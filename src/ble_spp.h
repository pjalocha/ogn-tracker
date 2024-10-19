
#include "fifo.h"

extern FIFO<char, 1024> BLE_SPP_TxFIFO;

void BLE_SPP_Check(void);
void BLE_SPP_Start(const char *DevName);

