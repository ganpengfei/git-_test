#ifndef _DRVFLASH_
#define _DRVFLASH_





INT8U DrvFlash_Erase(INT32U EraseAdd,INT32U EraseSize);
INT8U DrvFlash_WriteBuff(INT32U WriteAdd ,INT32U* Buff,INT8U Length);
INT8U DrvFlash_ReadBuff(INT32U WriteAdd ,INT32U* Buff,INT8U Length);












#endif
