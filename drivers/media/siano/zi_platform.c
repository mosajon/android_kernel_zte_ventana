/*******************************************************************************
** File Name: zi_platform.h
** Copyright: 2007-2012 Nationz Technologies Corp. All rights reserved.
** Version: 1.0
** Description: This file define the platform interface.
********************************************************************************/

#ifdef _cplusplus
	extern "C"
	{
#endif

#include "zi_platform.h"
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>

Zi_VOID *Zi_memcpy(Zi_VOID *dst, CONST Zi_VOID *src, Zi_INT32 len)
{
	/* add platform memcpy function */
	return (Zi_VOID *)memcpy(dst, src, len);
}

///////////////////////////////////////////////////////////////////////////////
Zi_VOID *Zi_memset(Zi_VOID *dst, Zi_INT32 c, Zi_INT32 len)
{
	/* add platform memset function */
	return (Zi_VOID *)memset(dst, c, len);
}

///////////////////////////////////////////////////////////////////////////////
Zi_INT32 Zi_memcmp(CONST Zi_VOID *mem1, CONST Zi_VOID *mem2, Zi_INT32 len)
{
	/* add platform memcmp function */
	return (Zi_INT32)memcmp(mem1, mem2, len);
}

///////////////////////////////////////////////////////////////////////////////
Zi_VOID *Zi_strcpy(Zi_VOID * dst,CONST Zi_VOID * src)
{
	/* add platform strcpy function */
	return (Zi_VOID *)strcpy(dst, src);
}

///////////////////////////////////////////////////////////////////////////////
Zi_INT32 Zi_strcmp(CONST Zi_VOID *str1, CONST Zi_VOID *str2)
{
	/* add platform strcmp function */
	return (Zi_INT32)strcmp(str1, str2);
}

///////////////////////////////////////////////////////////////////////////////
Zi_INT32 Zi_strlen(CONST Zi_VOID *str)
{
	/* add platform strlen function */
	return (Zi_INT32)strlen(str);
}


///////////////////////////////////////////////////////////////////////////////
Zi_VOID *Zi_strstr(CONST Zi_VOID *str1, CONST Zi_VOID *str2)
{
	/* add platform strstr function */
	return (Zi_VOID *)strstr(str1, str2);
}

///////////////////////////////////////////////////////////////////////////////
Zi_VOID *Zi_strcat(CONST Zi_VOID *str1, CONST Zi_VOID *str2)
{
	/* add platform strstr function */
	return (Zi_VOID *)strcat(str1, str2);
}

///////////////////////////////////////////////////////////////////////////////
Zi_VOID *Zi_alloc(Zi_UINT32 msize)
{
	/* add platform memory allocate  function */
	return (Zi_VOID *)kmalloc(msize, GFP_KERNEL);
}


///////////////////////////////////////////////////////////////////////////////
Zi_VOID Zi_free(Zi_VOID *pmem)
{
	/* add platform memory free  function */
	kfree(pmem);
}


///////////////////////////////////////////////////////////////////////////////
Zi_VOID Zi_msleep(Zi_UINT32 ms)
{
	/* add platform msleep  function */
	msleep(ms);
}

///////////////////////////////////////////////////////////////////////////////
Zi_CHAR Zi_PrintMsg_Buf[1024] = {0};
extern int sprintf(char *, const char*, ...);
Zi_VOID Zi_PrintMsg(Zi_CHAR *pBuf, Zi_UINT32 len, Zi_UINT8 iType)
{
	Zi_UINT8 i = 0;
	Zi_CHAR *tmp_buf = Zi_PrintMsg_Buf;
	
	if(len > 256)
	{
		Zi_log("The length of MsgData > 256.\n");
		return;
	}
	Zi_memset(tmp_buf, 0, 256);
	for(i = 0; i < len; i++)
	{
		tmp_buf += sprintf(tmp_buf, "%02x ", *(pBuf + i));
	}
	if(0 == iType)
	{
		Zi_log("Receive MSG data = %s\n", Zi_PrintMsg_Buf);
	}
	else if(1 == iType)
	{
		Zi_log("Send MSG data = %s\n", Zi_PrintMsg_Buf);
	}
	else if(2 == iType)
	{
		Zi_log("Receive 7816 data = %s\n", Zi_PrintMsg_Buf);
		Zi_log("-------------------Receive-------------------");
	}
	else if(3 == iType)
	{
		Zi_log("++++++++++++++++Send+++++++++++++++");
		Zi_log("Send 7816 data = %s\n", Zi_PrintMsg_Buf);
	}
	else if(4 == iType)
	{
		Zi_log("Debug data = %s\n", Zi_PrintMsg_Buf);
		
	}
}

///////////////////////////////////////////////////////////////////////////////
extern void zi_DataSpiTxRx(unsigned char *txbuf, unsigned char *rxbuf, int len);
Zi_RETURN_CODE Zi_SpiTxRx(Zi_UINT32 trxLen, Zi_UINT8 *pDataTx, Zi_UINT32 revLen, Zi_UINT8 *pDataRx )
{
	if(trxLen > revLen)
	{
		Zi_log("revLen < trxLen err!!!\n");
		return Zi_GENERAL_ERROR;
	}
	zi_DataSpiTxRx(pDataTx, pDataRx, revLen);
	
	//Zi_PrintMsg(pDataTx, trxLen, 1);
	//Zi_PrintMsg(pDataRx, revLen, 0);
	
	return Zi_RTN_OK;	
}

#ifdef   _cplusplus
	}
#endif








