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

#ifndef __MT9V114_H__
#define __MT9V114_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MT9V114_IOCTL_SET_MODE		_IOW('o', 1, struct mt9v114_mode)
#define MT9V114_IOCTL_GET_STATUS		_IOR('o', 2, struct mt9v114_status)
/* ZTE: modify by yaoling for yuv balance,EFFECT,scene 20110812 ++ */
#define MT9V114_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, enum mt9v114_coloreffect_mode)
#define MT9V114_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4,enum mt9v114_balance_mode)
#define MT9V114_IOCTL_SET_EXPOSURE       _IOW('o', 8, enum mt9v114_exposure_mode)
/* ZTE: modify by yaoling for yuv balance,EFFECT,scene 20110812 -- */
/* ZTE: add by yaoling for MT9V114 brightness 20110930 ++ */
#define MT9V114_IOCTL_SET_BRIGHTNESS  _IOW('o', 5, enum mt9v114_Brightness_mode)
/* ZTE: add by yaoling for MT9V114 brightness 20110930 --*/
struct mt9v114_mode {
	int xres;
	int yres;
};

struct mt9v114_status {
	int data;
	int status;
};



/* ZTE: add  by yaoling for yuv balance,expose,scene  20110815 ++ */
enum {
        MT9V114_ColorEffect = 0,
        MT9V114_Whitebalance,
        MT9V114_SceneMode,
        MT9V114_Exposure,
        /* ZTE: add by yaoling for 9v114 brightness 20110930 */
         MT9V114_Brightness = 6
};
enum mt9v114_balance_mode{
        MT9V114_Whitebalance_Invalid = 0,
        MT9V114_Whitebalance_Auto,
        MT9V114_Whitebalance_Incandescent,
        MT9V114_Whitebalance_Daylight,
        MT9V114_Whitebalance_Fluorescent,
};

enum mt9v114_exposure_mode{
        MT9V114_Exposure_0,
        MT9V114_Exposure_1,
        MT9V114_Exposure_2,
        MT9V114_Exposure_Negative_1,
        MT9V114_Exposure_Negative_2,
 
};
enum mt9v114_coloreffect_mode{
        MT9V114_ColorEffect_Invalid = 0,
        MT9V114_ColorEffect_Aqua,
        MT9V114_ColorEffect_Blackboard,
        MT9V114_ColorEffect_Mono,
        MT9V114_ColorEffect_Negative,
        MT9V114_ColorEffect_None,
        MT9V114_ColorEffect_Posterize,
        MT9V114_ColorEffect_Sepia,
        MT9V114_ColorEffect_Solarize,
        MT9V114_ColorEffect_Whiteboard
};

/* ZTE: add  by yaoling for yuv balance,expose,scene  20110815 -- */
/* ZTE: add by yaoling for MT9V114 brightness 20110930 ++ */
enum mt9v114_Brightness_mode{
        mt9v114_Brightness_Level1= 0,
        mt9v114_Brightness_Level2,
        mt9v114_Brightness_Level3,
        mt9v114_Brightness_Level4,
        mt9v114_Brightness_Level5
};
/* ZTE: add by yaoling for MT9V114 brightness 20110930 --*/
#ifdef __KERNEL__
struct mt9v114_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __MT9V114_H__ */

