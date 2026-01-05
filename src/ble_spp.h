
#include "fifo.h"

extern FIFO<char, 2048> BLE_SPP_TxFIFO;
extern FIFO<char, 2048> BLE_SPP_RxFIFO;

void BLE_SPP_Check(void);
void BLE_SPP_Start(const char *DevName);

