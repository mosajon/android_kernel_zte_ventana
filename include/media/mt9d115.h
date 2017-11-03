/**
 * Copyright (c) 2011 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __MT9D115_H__
#define __MT9D115_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MT9D115_IOCTL_SET_MODE		_IOW('o', 1, struct mt9d115_mode)
#define MT9D115_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define SENSOR_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, int)
#define SENSOR_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4, int)
#define SENSOR_IOCTL_SET_SCENE_MODE     _IOW('o', 5, __u8)
#define SENSOR_IOCTL_SET_AF_MODE        _IOW('o', 6, __u8)
#define SENSOR_IOCTL_GET_AF_STATUS      _IOW('o', 7, __u8)
#define SENSOR_IOCTL_SET_EXPOSURE       _IOW('o', 8, int)
#define SENSOR_IOCTL_SET_BRIGHTNESS       _IOW('o', 9, __u8)
#define SENSOR_IOCTL_SET_ISO       _IOW('o', 10, __u8)
#define SENSOR_IOCTL_SET_Contrast        _IOW('o', 11, __u8)
#define SENSOR_IOCTL_SET_Saturation        _IOW('o', 12, __u8)
#define SENSOR_IOCTL_SET_Sharpness        _IOW('o', 13, __u8)
enum {
      YUV_ColorEffect = 0,
      YUV_Whitebalance,
      YUV_SceneMode,
      YUV_Exposure,
      YUV_Brightness,
      YUV_ISO,
      YUV_Contrast,
      YUV_Saturation,
      YUV_Sharpness
};

enum {
      YUV_ColorEffect_Invalid = 0,
      YUV_ColorEffect_Aqua,
      YUV_ColorEffect_Blackboard,
      YUV_ColorEffect_Mono,
      YUV_ColorEffect_Negative,
      YUV_ColorEffect_None,
      YUV_ColorEffect_Posterize,
      YUV_ColorEffect_Sepia,
      YUV_ColorEffect_Solarize,
      YUV_ColorEffect_Whiteboard,
      YUV_ColorEffect_vivid,
      YUV_ColorEffect_Emboss,
      YUV_ColorEffect_redtint,
      YUV_ColorEffect_bluetint,
      YUV_ColorEffect_greentint
};

enum {
      YUV_Whitebalance_Invalid = 0,
      YUV_Whitebalance_Auto,
      YUV_Whitebalance_Incandescent,
      YUV_Whitebalance_Fluorescent,
      YUV_Whitebalance_WarmFluorescent,
      YUV_Whitebalance_Daylight,
      YUV_Whitebalance_CloudyDaylight,
      YUV_Whitebalance_Shade,
      YUV_Whitebalance_Twilight,
      YUV_Whitebalance_Custom
};

enum {
      YUV_SceneMode_Invalid = 0,
      YUV_SceneMode_Auto,
      YUV_SceneMode_Action,
      YUV_SceneMode_Portrait,
      YUV_SceneMode_Landscape,
      YUV_SceneMode_Beach,
      YUV_SceneMode_Candlelight,
      YUV_SceneMode_Fireworks,
      YUV_SceneMode_Night,
      YUV_SceneMode_NightPortrait,
      YUV_SceneMode_Party,
      YUV_SceneMode_Snow,
      YUV_SceneMode_Sports,
      YUV_SceneMode_SteadyPhoto,
      YUV_SceneMode_Sunset,
      YUV_SceneMode_Theatre,
      YUV_SceneMode_Barcode
};
enum {
      YUV_Exposure_Negative_2 = -2,
      YUV_Exposure_Negative_1,
      YUV_Exposure_0,
      YUV_Exposure_1,
      YUV_Exposure_2
};
enum {
      YUV_Brightness_0,
      YUV_Brightness_1,
      YUV_Brightness_2,
      YUV_Brightness_3,
      YUV_Brightness_4
};

enum {
      YUV_ISO_AUTO,
      YUV_ISO_100,
      YUV_ISO_200,
      YUV_ISO_400 = 4,
      YUV_ISO_800 = 8,
      YUV_ISO_1600 = 16
};

enum {
      YUV_Contrast_0,
      YUV_Contrast_1,
      YUV_Contrast_2,
      YUV_Contrast_3,
      YUV_Contrast_4
};

enum {
      YUV_Saturation_0,
      YUV_Saturation_1,
      YUV_Saturation_2,
      YUV_Saturation_3,
      YUV_Saturation_4
};

enum {
      YUV_Sharpness_0,
      YUV_Sharpness_1,
      YUV_Sharpness_2,
      YUV_Sharpness_3,
      YUV_Sharpness_4
};
struct mt9d115_mode {
	int xres;
	int yres;
};
#ifdef __KERNEL__
struct md9d115_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __MT9D115_H__ */

