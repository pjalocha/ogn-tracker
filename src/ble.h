
#include "fifo.h"

extern FIFO<char, 1024> BLE_TxFIFO;

void BLE_Check(void);
void BLE_Start(const char *DevName);

