/******************************************************************************
 *	File	:	EcmUsrDriver.h
 *	Version :	0.6
 *	Date	:	2020/11/20
 *	Author	:	XFORCE
 *
 *	ECM-XF basic driver example - Header file
 *
 *	Demonstrate how to implement API type user driver
 *
 * @copyright (C) 2020 NEXTW TECHNOLOGY CO., LTD.. All rights reserved.
 *
 ******************************************************************************/
#include "EcmUsrDriver.h"
#include "EcmGpioDriver.h"

extern SPI_CMD_PACKAGE_T *pCmd;
extern SPI_RET_PACKAGE_T *pRet;
extern uint8_t u8CmdIdx;

int ECM_GpioSetMode(uint8_t ch, uint8_t direction, uint8_t pusel)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_GPIO_CONFIG_SET;
	pCmd->Head.u8Param = 0;
	pCmd->Head.u16Size = 0;
	if (ch < 8)
	{
		pCmd->Head.u8Data[0] = 1u << ch;
		pCmd->Head.u8Data[1] = 0;
	}
	else if (ch < 16)
	{
		pCmd->Head.u8Data[1] = 1u << (ch - 8);
		pCmd->Head.u8Data[0] = 0;
	}
	else if (ch < 20)
	{
		pCmd->Head.u8Param = 8;
		pCmd->Head.u8Data[0] = 1u << (ch - 16);
		pCmd->Head.u8Data[1] = 0;
	}
	else
		return -1;
	pCmd->Head.u8Data[2] = direction & 3u;
	pCmd->Head.u8Data[3] = pusel & 3u;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return 1;
			}
		}
	}
	return 0;
}

int ECM_GpioEnableDebounce(uint8_t ch, uint8_t enable)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_GPIO_CONFIG_SET;
	pCmd->Head.u8Param = 2;
	pCmd->Head.u16Size = 0;
	if (ch < 8)
	{
		pCmd->Head.u8Data[0] = 1u << ch;
		pCmd->Head.u8Data[1] = 0;
	}
	else if (ch < 16)
	{
		pCmd->Head.u8Data[1] = 1u << (ch - 8);
		pCmd->Head.u8Data[0] = 0;
	}
	else if (ch < 20)
	{
		pCmd->Head.u8Param = 9;
		pCmd->Head.u8Data[0] = 1u << (ch - 16);
		pCmd->Head.u8Data[1] = 0;
	}
	else
		return -1;
	pCmd->Head.u8Data[2] = enable & 1u;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return 1;
			}
		}
	}
	return 0;
}

int ECM_GpioSetDebounceClock(uint8_t source, uint8_t clock)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_GPIO_CONFIG_SET;
	pCmd->Head.u8Param = 3;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Data[0] = (source & 1u) << 4 | (clock & 0xfu);
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return 1;
			}
		}
	}
	return 0;
}

int ECM_GpioSetValue(uint16_t val)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_GPIO_FUNC_OP;
	pCmd->Head.u8Param = 0;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Data[0] = val & 0xff;
	pCmd->Head.u8Data[1] = (val >> 8) & 0xff;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return 1;
			}
		}
	}
	return 0;
}

int ECM_GpioExtSetValue(uint32_t val)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_GPIO_FUNC_OP;
	pCmd->Head.u8Param = 2;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Data[0] = val & 0xff;
	pCmd->Head.u8Data[1] = (val >> 8) & 0xff;
	pCmd->Head.u8Data[2] = (val >> 16) & 0xff;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return 1;
			}
		}
	}
	return 0;
}

int ECM_GpioGetValue(uint16_t *val)
{
	int i = 0;
	uint8_t IdxCheck;
	if (!val)
		return -1;
	pCmd->Head.u8Cmd = ECM_GPIO_FUNC_OP;
	pCmd->Head.u8Param = 1;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				*val = pRet->Data[0] | (pRet->Data[1] << 8);
				return 1;
			}
		}
	}
	return 0;
}

int ECM_GpioExtGetValue(uint32_t *val)
{
	int i = 0;
	uint8_t IdxCheck;
	if (!val)
		return -1;
	pCmd->Head.u8Cmd = ECM_GPIO_FUNC_OP;
	pCmd->Head.u8Param = 3;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				*val = pRet->Data[0] | (pRet->Data[1] << 8) | (pRet->Data[2] << 16);
				return 1;
			}
		}
	}
	return 0;
}

int ECM_GpioIntEnable(uint8_t ch, uint8_t inttype)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_GPIO_CONFIG_SET;
	pCmd->Head.u8Param = 1;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	if (ch < 20 && inttype < 6)
	{
		pCmd->Head.u8Data[0] = ch;
		pCmd->Head.u8Data[1] = inttype;
	}
	else
	{
		return -1;
	}
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return 1;
			}
		}
	}
	return 0;
}

int ECM_GpioIntClear(uint8_t ch)
{
	int i = 0;
	uint8_t IdxCheck;
	pCmd->Head.u8Cmd = ECM_GPIO_CONFIG_SET;
	pCmd->Head.u8Param = 4;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	if (ch < 8)
	{
		pCmd->Head.u8Data[0] = 1u << ch;
		pCmd->Head.u8Data[1] = 0;
	}
	else if (ch < 16)
	{
		pCmd->Head.u8Data[1] = 1u << (ch - 8);
		pCmd->Head.u8Data[0] = 0;
	}
	else if (ch < 20)
	{
		pCmd->Head.u8Param = 6;
		pCmd->Head.u8Data[0] = 0;
		pCmd->Head.u8Data[1] = 0;
		pCmd->Head.u8Data[2] = 1u << (ch - 16);
	}
	else
	{
		return -1;
	}
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				return 1;
			}
		}
	}
	return 0;
}

int ECM_GpioGetIntFlag(uint16_t *val)
{
	int i = 0;
	uint8_t IdxCheck;
	if (!val)
		return -1;
	pCmd->Head.u8Cmd = ECM_GPIO_FUNC_OP;
	pCmd->Head.u8Param = 5;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				*val = pRet->Data[0] | (pRet->Data[1] << 8);
				return 1;
			}
		}
	}
	return 0;
}

int ECM_GpioExtGetIntFlag(uint32_t *val)
{
	int i = 0;
	uint8_t IdxCheck;
	if (!val)
		return -1;
	pCmd->Head.u8Cmd = ECM_GPIO_FUNC_OP;
	pCmd->Head.u8Param = 7;
	pCmd->Head.u16Size = 0;
	pCmd->Head.u8Idx = u8CmdIdx++;
	for (i = 0; i < 100; i++)
	{
		if (SpiDataExchange(&IdxCheck, 0))
		{
			if (pCmd->Head.u8Idx == IdxCheck)
			{
				*val = pRet->Data[0] | (pRet->Data[1] << 8 | pRet->Data[2] << 16);
				return 1;
			}
		}
	}
	return 0;
}
