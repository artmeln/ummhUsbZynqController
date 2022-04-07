#include "xusbps.h"		/* USB controller driver */

int SetupUsbDevice(u8* pTtxEventCounterEp2);
void ReadFromEp1(u8** buff, u8* length);
void ResetRxBufferEp1();
int SendToEp1(u8* buff, u8 length);
int SendToEp2(u8* buff, u32 length);

