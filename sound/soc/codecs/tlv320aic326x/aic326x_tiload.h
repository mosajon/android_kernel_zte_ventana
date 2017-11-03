/*
 * linux/sound/soc/codecs/aic3262_tiload.h
 *
 *
 * Copyright (C) 2011 Mistral Solutions Pvt Ltd.
 *
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * 
 *
 * 
 */

#ifndef _AIC3262_TILOAD_H
#define _AIC3262_TILOAD_H

/* typedefs required for the included header files */
typedef char *string;

/* defines */
#define DEVICE_NAME				"tiload_node"
#define aic3262_IOC_MAGIC		0xE0
#define aic3262_IOMAGICNUM_GET	_IOR(aic3262_IOC_MAGIC, 1, int)
#define aic3262_IOMAGICNUM_SET	_IOW(aic3262_IOC_MAGIC, 2, int)
#define aic3262_DUMPREG_GET		_IOR(aic3262_IOC_MAGIC, 3, int)
#define aic3262_IODSP_NUM_GET	_IOR(aic3262_IOC_MAGIC, 4, int)
#define aic3262_IODSP1_CODE_GET	_IOR(aic3262_IOC_MAGIC, 5, int)
#define aic3262_IODSP2_CODE_GET	_IOR(aic3262_IOC_MAGIC, 6, int)

#endif