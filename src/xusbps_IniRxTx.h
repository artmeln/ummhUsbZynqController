#include "xusbps.h"		/* USB controller driver */

int SetupUsbDevice();
void ReadFromEp1(u8** buff, u8* length);
void ReadFromEp2(u8** buff, u8* length);
void ResetRxBufferEp1();
void ResetRxBufferEp2();
int SendToEp1(u8* buff, u8 length);
int SendToEp2(u8* buff, u8 length);

