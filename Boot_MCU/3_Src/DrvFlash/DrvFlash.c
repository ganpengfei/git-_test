#include "include.h"







/***********************************************************************************
*Name      : DrvFlash_Erase
*Brief  	 :  擦缓存扇区
*Parameter : INT32U EraseAdd
							INT32U EraseSize
*return    : 擦除成功或失败，TRUE or FALSE
***********************************************************************************/
INT8U DrvFlash_Erase(INT32U EraseAdd,INT32U EraseSize)
{
	flash_config_t flashDriver;                                            /* Flash driver Structure */
	flash_security_state_t securityStatus = kFLASH_securityStateNotSecure; /* Return protection status */
	status_t result;    /* Return code from each flash driver function */
	INT32U MaskValue;

	MaskValue = DisableGlobalIRQ();
	/* Clean up Flash driver Structure*/
	memset(&flashDriver, 0, sizeof(flash_config_t));
	/*Setup flash driver structure for device and initialize variables. */
	result = FLASH_Init(&flashDriver);
	if (kStatus_FLASH_Success != result)
	{
		return FAIL;//FLASH error
	}
	/* Check security status. */		
	result = FLASH_GetSecurityState(&flashDriver, &securityStatus);
	if ((kStatus_FLASH_Success != result) || (securityStatus != kFLASH_securityStateNotSecure))
	{
		 return FAIL;//FLASH error
	}
	result = FLASH_Erase(&flashDriver, EraseAdd,EraseSize, kFLASH_apiEraseKey);
	if (kStatus_FLASH_Success != result)
	{
			return FAIL;//FLASH error
	}
	/* Verify sector if it's been erased. */
	result = FLASH_VerifyErase(&flashDriver, EraseAdd,EraseSize, kFLASH_marginValueUser);
	if (kStatus_FLASH_Success != result)
	{
			return FAIL;//FLASH error
	}
	EnableGlobalIRQ(MaskValue);
	return SUCC;
}

/***********************************************************************************
*Name      : DrvFlash_WriteBuff
*Brief  	 : 写数据
*Parameter :  INT32U WriteAdd 写缓存
							INT32U* Buff 写数据地址 
							INT8U Length 数据长度   per bytes
*return    : 写成功或失败，TRUE or FALSE
***********************************************************************************/
INT8U DrvFlash_WriteBuff(INT32U WriteAdd ,INT32U* Buff,INT8U Length)
{
	flash_config_t flashDriver;   
	INT32U MaskValue;
	status_t result;    /* Return code from each flash driver function */
	flash_security_state_t securityStatus = kFLASH_securityStateNotSecure; /* Return protection status */
	
	MaskValue = DisableGlobalIRQ();
	/* Clean up Flash driver Structure*/
	memset(&flashDriver, 0, sizeof(flash_config_t));
	/*Setup flash driver structure for device and initialize variables. */
	result = FLASH_Init(&flashDriver);
	if (kStatus_FLASH_Success != result)
	{
		return FAIL;//FLASH error
	}
	/* Check security status. */		
	result = FLASH_GetSecurityState(&flashDriver, &securityStatus);
	if ((kStatus_FLASH_Success != result) || (securityStatus != kFLASH_securityStateNotSecure))
	{
		 return FAIL;//FLASH error
	}
	
	if(Length % 8)
	{
		return FAIL;//FLASH error
	}

		result = FLASH_Program(&flashDriver, WriteAdd, &Buff[0], Length);
	if (kStatus_FLASH_Success != result)
	{
			return FAIL;//FLASH error
	}
	result = FLASH_VerifyProgram(&flashDriver, WriteAdd, Length, &Buff[0],
																kFLASH_marginValueUser, NULL,NULL);
	if (kStatus_FLASH_Success != result)
	{
			return FAIL;//FLASH error
	}		

	EnableGlobalIRQ(MaskValue);
	return SUCC;
}


/***********************************************************************************
*Name      : DrvFlash_ReadBuff
*Brief  	 : 读一段地址的数据
*Parameter :  INT32U WriteAdd 写缓存
							INT32U* Buff 写数据地址 
							INT8U Length 数据长度   per bytes
*return    : 写成功或失败，TRUE or FALSE
***********************************************************************************/
INT8U DrvFlash_ReadBuff(INT32U WriteAdd ,INT32U* Buff,INT8U Length)
{
	INT32U i;
	for(i=0;i<(Length/4);i++)
	{
		Buff[i] = *(INT32U*)(WriteAdd + i*4);
	}
	return SUCC;
}



