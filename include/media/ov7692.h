/*
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __OV7692_H__
#define __OV7692_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define OV7692_IOCTL_SET_MODE		_IOW('o', 1, struct ov7692_mode)
#define OV7692_IOCTL_GET_STATUS		_IOR('o', 2, struct ov7692_status)
/* ZTE: modify by yaoling for yuv balance,EFFECT,scene 20110812 ++ */
#define OV7692_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3,  int)
#define OV7692_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4, int)
#define OV7692_IOCTL_SET_EXPOSURE       _IOW('o', 8,  int)
/* ZTE: modify by yaoling for yuv balance,EFFECT,scene 20110812 -- */
/* ZTE: add by yaoling for OV7692 brightness 20110930 ++ */
#define OV7692_IOCTL_SET_BRIGHTNESS  _IOW('o', 5, enum ov7692_Brightness_mode)
/* ZTE: add by yaoling for OV7692 brightness 20110930 --*/
struct ov7692_mode {
	int xres;
	int yres;
};

struct ov7692_status {
	int data;
	int status;
};
enum {
        OV7692_ColorEffect = 0,
        OV7692_Whitebalance,
        OV7692_SceneMode,
        OV7692_Exposure,
        /* ZTE: add by yaoling for 9v114 brightness 20110930 */
         OV7692_Brightness = 6
};

/* ZTE: add by yaoling for OV7692 brightness 20110930 ++ */
enum ov7692_Brightness_mode{
        ov7692_Brightness_Level1= 0,
        ov7692_Brightness_Level2,
        ov7692_Brightness_Level3,
        ov7692_Brightness_Level4,
        ov7692_Brightness_Level5
};
/* ZTE: add by yaoling for OV7692 brightness 20110930 --*/
#ifdef __KERNEL__
struct ov7692_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __OV7692_H__ */

