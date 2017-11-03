/*******************************************************************************
** File Name: zi_platform.h
** Copyright: 2007-2012 Nationz Technologies Corp. All rights reserved.
** Version: 1.0
** Description: This file define the platform interface.
********************************************************************************/
	
#ifndef _ZI_PLATFORM_H
#define _ZI_PLATFORM_H
	
#include "zi_types.h"

/*******************************************************************************
* Zi_memcpy - Copy one area of memory to another
* @dest: Where to copy to
* @src: Where to copy from
* @len: The size of the area.
********************************************************************************/
Zi_VOID *Zi_memcpy(Zi_VOID *dst, CONST Zi_VOID *src, Zi_INT32 len);

/*******************************************************************************
* Zi_memset - Fill a region of memory with the given value
* @dst: Pointer to the start of the area.
* @c: The byte to fill the area with
* @len: The size of the area.
*******************************************************************************/
Zi_VOID *Zi_memset(Zi_VOID *dst, Zi_INT32 c, Zi_INT32 len);

/*******************************************************************************
* Zi_memcmp - Compare two areas of memory
* @mem1: One area of memory
* @mem2: Another area of memory
* @len: The size of the area.
*******************************************************************************/
Zi_INT32 Zi_memcmp(CONST Zi_VOID *mem1, CONST Zi_VOID *mem2, Zi_INT32 len);

/*******************************************************************************
* Zi_strcpy - Copy a %NUL terminated string.
* @dest: Where to copy the string to
* @src: Where to copy the string from
*******************************************************************************/
Zi_VOID *Zi_strcpy(Zi_VOID * dst,CONST Zi_VOID * src);

/*******************************************************************************
* Zi_strcmp - Compare two strings
* @str1: One string
* @str2: Another string
*******************************************************************************/
Zi_INT32 Zi_strcmp(CONST Zi_VOID *str1, CONST Zi_VOID *str2);

/*******************************************************************************
* Zi_strlen - Find the length of a string
* @str: The string to be sized
********************************************************************************/
Zi_INT32 Zi_strlen(CONST Zi_VOID *str);

/*******************************************************************************
* Zi_strstr - find str2 in str1.
* @str1: The string to be searched.
* @str2: The string to search for.
********************************************************************************/
Zi_VOID *Zi_strstr(CONST Zi_VOID *str1, CONST Zi_VOID *str2);

/*******************************************************************************
* Zi_strcat - Append one %NUL-terminated string to another.
* @str1: The string to be appended to.
* @str2: The string to append to it.
********************************************************************************/
Zi_VOID *Zi_strcat(CONST Zi_VOID *str1, CONST Zi_VOID *str2);

/*******************************************************************************
* Zi_alloc - allocate memory
* @msize: how many bytes of memory are required.
*******************************************************************************/
Zi_VOID *Zi_alloc(Zi_UINT32 msize);

/*******************************************************************************
* Zi_free - free previously allocated memory
* @pmem: pointer returned by kmalloc.
*******************************************************************************/
Zi_VOID Zi_free(Zi_VOID *pmem);

/*******************************************************************************
* Zi_msleep - sleep safely even with waitqueue interruptions
* @ms: Time in milliseconds to sleep for
*******************************************************************************/
Zi_VOID Zi_msleep(Zi_UINT32 ms);

/*******************************************************************************
* Zi_PrintMsg - Print debug msg data
* @pBuf: pointer to the msg data
* @len: the length of the data
* @iType: the type of the data
*******************************************************************************/
Zi_VOID Zi_PrintMsg(Zi_CHAR *pBuf, Zi_UINT32 len, Zi_UINT8 iType);


/*******************************************************************************
* Zi_SpiTxRx - Tx/Rx
* @trelen: length to be txed
* @revlen: length to be rxed
* @pDataTx: pointer to the data to be txed
* @pDataRx: pointer to the data to be rxed
*******************************************************************************/
Zi_RETURN_CODE Zi_SpiTxRx(Zi_UINT32 trxlen, Zi_UINT8 *pDataTx, Zi_UINT32 revlen, Zi_UINT8 *pDataRx);
	
#endif /* _ZI_PLATFORM_H */

