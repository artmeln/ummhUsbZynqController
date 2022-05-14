#include "xusbps.h"		/* USB controller driver */

int SetupUsbDevice(u8* pTxEventCounterEp2, u8* pTxEventCounterEp3);
void ReadFromEp1(u8** buff, u8* length);
void ResetRxBufferEp1();
int SendToEp1(u8* buff, u8 length);
int SendToEp2(u8* buff, u32 length);
int SendToEp3(u8* buff, u32 length);

