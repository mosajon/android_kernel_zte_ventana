/*******************************************************************************
** File Name: zi_types.h
** Copyright: 2007-2012 Nationz Technologies Corp. All rights reserved.
** Version: 1.0
** Description: This file contains predefined data types for CMMB 
**              software development
********************************************************************************/

#ifndef _ZI_TYPES_H
#define _ZI_TYPES_H

#ifdef __cplusplus
	extern "C"
	{
#endif

#define CONST			const

#define TRUE			1
#define FALSE			0

#ifndef NULL
#define NULL			0
#endif

typedef void			Zi_VOID;
typedef void *			Zi_PVOID;

typedef unsigned char		Zi_BOOL;

typedef char			Zi_INT8;
typedef char			Zi_CHAR;
typedef unsigned char		Zi_UINT8;

typedef short			Zi_SHORT;
typedef unsigned short		Zi_USHORT;

typedef short			Zi_INT16;
typedef unsigned short		Zi_UINT16;


typedef int			Zi_INT;
typedef unsigned int		Zi_UINT;

typedef int			Zi_INT32;
typedef unsigned int		Zi_UINT32;

typedef float			Zi_FLOAT;

typedef double			Zi_DOUBLE;

typedef unsigned long		Zi_ULONG;

//#define Zi_CMMB_DEBUG_SWITCH 1

#include <linux/kernel.h>

#define Zi_printk1(kern, fmt, arg...) \
	printk(kern "[zm2167] %s: " fmt "\n", __func__, ##arg)

#define Zi_printk2(kern, fmt, arg...) \
	printk(kern "[Debug] %s: " fmt "\n", __func__, ##arg)

#ifdef Zi_CMMB_DEBUG_SWITCH

#define Zi_log(fmt, arg...) Zi_printk1(KERN_ALERT, fmt, ##arg)
#define Zi_logg(fmt, arg...) Zi_printk2(KERN_ALERT, fmt, ##arg)
#else
#define Zi_log(fmt, arg...)
#define Zi_logg(fmt, arg...) //Zi_printk2(KERN_ALERT, fmt, ##arg)
#endif
	
typedef enum {
	Zi_RTN_OK = 0,
	Zi_GENERAL_ERROR = -1000,
	Zi_TIMEOUT_ERROR,
	Zi_PARAMETER_ERROR,
	Zi_POLLING_STATUS,
	Zi_FTS_FAIL_INT,
	Zi_FTS_FAIL_CURRENT,
	Zi_TIME_OUT,
	Zi_POOR_SIGNAL,
	Zi_CLCHCRC_FAIL,
	Zi_SYSTEM_UNLOCK,
} Zi_RETURN_CODE;

#ifdef   __cplusplus
	}
#endif

#endif /* _ZI_TYPES_H */



