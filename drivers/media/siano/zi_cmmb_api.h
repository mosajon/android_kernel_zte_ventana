#ifndef _ZI_CMMB_API_H
#define _ZI_CMMB_API_H

#include "zi_types.h"
#include "zi_platform.h"
#include "zi_communication.h"

#define Zi_CMMB_SDK_VERSION	"V1.0 $Rev: 13 $"
#define Zi_CMMB_STATS_NUM_CHANNELS	6

#define CMMB_GETTS0_PARSER_FLAG	(1 << 4)     /* ts0 paser one time for nationz chip*/
#define CMMB_INT_PROCESS_FLAG		(1<<3)
#define CMMB_DEVICE_RELEASE_FLAG	(1<<2)

typedef enum
{
	Zi_CTRL_CMD_TYPE_SWITCH_MODE			= 1,
	Zi_CTRL_CMD_TYPE_SET_FREQ			= 2,
	Zi_CTRL_CMD_TYPE_GET_SIGNAL_POWER		= 3,
	Zi_CTRL_CMD_TYPE_SYS_CTRL			= 4,
	Zi_CTRL_CMD_TYPE_GET_REG			= 5,
	Zi_CTRL_CMD_TYPE_SET_REG			= 6,
	Zi_CTRL_CMD_TYPE_UPDATE_FIRMWARE_FLASH		= 7,
	Zi_CTRL_CMD_TYPE_GET_FIRMWARE_VERSION		= 8,
	Zi_CTRL_CMD_TYPE_READ_FLASH			= 9,
	Zi_CTRL_CMD_TYPE_TUNER_CONFIG			= 0x0A,
	Zi_CTRL_CMD_TYPE_REGISTER_READANDWRITER 	= 0x0B,
	Zi_CTRL_CMD_TYPE_ISO7816_OPERATION		= 0x0C,
	Zi_CTRL_CMD_TYPE_START_THRESHOLD		= 0x0D,
	Zi_CTRL_CMD_TYPE_SET_MSG_PARA			= 0x0E,
	Zi_CTRL_CMD_TYPE_GET_CA_VENDEOR_INFO		= 0x0F,
	Zi_CTRL_CMD_TYPE_GET_CA_VERSION			= 0x10,
	Zi_CTRL_CMD_TYPE_GET_CA_SID			= 0x11,
	Zi_CTRL_CMD_TYPE_GET_CA_SN			= 0x12,
	Zi_CTRL_CMD_TYPE_START_STOP_REG_TRACE		= 0x13,
	Zi_CTRL_CMD_TYPE_SET_LOGIC_CHANNEL		= 0x20,
	Zi_CTRL_CMD_TYPE_CLOSE_LOGIC_CHANNEL		= 0x21,
	Zi_CTRL_CMD_TYPE_LOGIC_CHANNEL_OPERATION	= 0x22,
	Zi_CTRL_CMD_TYPE_SERVICE_OPEN			= 0x23,
	Zi_CTRL_CMD_TYPE_SERVICE_CLOSE			= 0x24,
	Zi_CTRL_CMD_TYPE_SEND_CW			= 0x25,
	Zi_CTRL_CMD_TYPE_CLOSE_ALL_LOGIC_CHANNEL	= 0x26,
	Zi_CTRL_CMD_TYPE_ADD_PHYSICAL_CHANNEL		= 0x50,
	Zi_CTRL_CMD_TYPE_DEL_PHYSICAL_CHANNEL		= 0x51,
	Zi_CTRL_CMD_TYPE_SCAN_FREQ			= 0x60,
	Zi_CTRL_CMD_TYPE_NCMD_ISO7816_OPERATION		= 0x61,
	Zi_CTRL_CMD_TYPE_INIT				= 0x71,

}Zi_CTRL_CMD_TYPE_E;

#define MF_FRAME_START_TAG	"\x00\x00\x00\x01"

typedef struct
{
	Zi_UINT8	CmdType;
	//cmd
	Zi_UINT8	CmdResponse;
	Zi_UINT8	CmdResult;

	//data

	Zi_UINT8	ChannelNumber;
	Zi_UINT32	DataLen;
	Zi_UINT8	*Data;
	Zi_UINT32	DataLenMax;

}Zi_CMD_DATA_T;



typedef enum
{
	Zi_FRAME_TYPE_MULTIPLEX_FRAME_DATA		= 0x01,
	Zi_FRAME_TYPE_COMMAND_RESPONSE			= 0x10,
	Zi_FRAME_TYPE_COMMAND_ACK			= 0x11,
	Zi_FRAME_TYPE_REGISTER_READ			= 0x20,
	Zi_FRAME_TYPE_ISO7816_READ			= 0x21,
	Zi_FRAME_TYPE_REG_TRACE				= 0x22,
	Zi_FRAME_TYPE_IR_RECEIVE_MSG			= 0x30,
	Zi_FRAME_TYPE_RECEIVE_THRD_MSG			= 0x31,
	Zi_FRAME_TYPE_DEMOD_MSG				= 0x32,
	Zi_FRAME_TYPE_SIGNAL_STATE1_MSG			= 0x33,
	Zi_FRAME_TYPE_SIGNAL_STATE2_MSG			= 0x34,
	Zi_FRAME_TYPE_DEBUG_MSG				= 0x35,
	Zi_FRAME_TYPE_CA_MSG				= 0x36,
	Zi_FRAME_TYPE_SIGNAL_STATE3_MSG			= 0x37,
	Zi_FRAME_TYPE_7816_NCMD_RESPONSE		= 0x60,
}Zi_FRAME_TYPE_E;

typedef struct
{
	Zi_UINT8 ChannelId;
	Zi_UINT16 ServiceId;
}Zi_CMMB_CHANNEL_AND_SERVICE_ID;

typedef struct
{
	Zi_CMMB_CHANNEL_AND_SERVICE_ID ChannelInfo[Zi_CMMB_STATS_NUM_CHANNELS];
	Zi_UINT8 ChannelInfoId;
}Zi_CMMB_CONTEXT;


#define CMMB_UNKOWN_DEVICE	0
#define CMMB_SMS1186_DEVICE	2
#define CMMB_SMS1180_DEVICE	4
#define CMMB_ZM2167_DEVICE	5
////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_Chip_Test(Zi_VOID);

////////////////////////////////////////////////////////////////////////
Zi_HANDLE Zi_OpenDevice(Zi_VOID);

////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_CmdSetFreq(Zi_UINT32 freqpoint);

////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_Cmmb_Select_Service(Zi_UINT16 serviceId);

////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_Cmmb_Stop_Service(Zi_UINT16 serviceId);

////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_CmdISO7816Reset(Zi_VOID);

////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_CmdAPDU(Zi_UINT8 *APDUCmd, Zi_UINT32 CmdLength);

////////////////////////////////////////////////////////////////////////
Zi_VOID Zi_CmdProcess(Zi_HANDLE hDevice);

////////////////////////////////////////////////////////////////////////
Zi_VOID Zi_CLCHDataParser(Zi_HANDLE hDevice, Zi_CMD_DATA_T *CmdData);

////////////////////////////////////////////////////////////////////////
Zi_BOOL Zi_IsMpxFrameHeader( Zi_UINT8 *pBuf, Zi_UINT32 BufSize, Zi_UINT32 *pOutFrameSize );

////////////////////////////////////////////////////////////////////////
Zi_UINT32 Zi_CRC32Compute(Zi_UINT32 InitialVector, Zi_UINT8 *pData, Zi_UINT32 Len);

////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE DMX_Initialize(Zi_HANDLE hDevice);

////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE DMX_Destory(Zi_HANDLE hDevice);



////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_CloseDevice(Zi_HANDLE hDevice);



#endif

