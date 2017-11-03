
#include "zi_cmmb_api.h"
#include <linux/slab.h>

extern Zi_CMD_DATA_T g_ziCmdData;
extern Zi_VOID Zi_irq_level(Zi_UINT8 iplace);

extern Zi_UINT8 *Zi_dev_area;
Zi_CMD_DATA_T Zi_CmdData;
Zi_UINT8 DataCount=0;

Zi_DEVICE_HANDLE *Zi_pCmmbDevice = NULL; 

Zi_UINT32 Zi_cmmb_state = 0;

Zi_UINT8 Zi_CmmbCmdData[32]={0};
Zi_UINT8 Zi_CmmbFrameHeadrData[32]={0};
Zi_UINT8 Zi_Cmmb7816Data[32]={0};

Zi_UINT8 g_7816cmd[256];
Zi_UINT8 g_7816bufLen;

Zi_INT32 cmmb_chip_type = CMMB_UNKOWN_DEVICE;


extern int sprintf(char *, const char*, ...);
extern void  smschar_interrupt(void);


Zi_DEVICE_CONFIG Zi_DeviceConfig = 
{ 
	Zi_SPI,  /*eControlPath*/
	Zi_SPI,  /*eDataPath*/
	0,  /*nIntType*/
	Zi_INT_LOW_ACTIVE,  /*eIntLevel*/
};

///////////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_Chip_Test(Zi_VOID)
{
	Zi_RETURN_CODE eRtnCode = Zi_GENERAL_ERROR;
	Zi_DEVICE_HANDLE *pDevice = NULL;
    
	pDevice = (Zi_DEVICE_HANDLE *)Zi_alloc(sizeof(Zi_DEVICE_HANDLE));
	if(!pDevice)
	{
		eRtnCode = Zi_GENERAL_ERROR;
		goto CHIP_TEST_RET;
	}
	Zi_memset(pDevice, 0, sizeof(Zi_DEVICE_HANDLE));

	pDevice->pDevRxBuffer = (Zi_UINT8 *)kmalloc(SPI_BURST_LEN + 256, GFP_KERNEL | __GFP_DMA);
	pDevice->pDevTxBuffer = (Zi_UINT8 *)kmalloc(SPI_BURST_LEN + 256, GFP_KERNEL | __GFP_DMA);
	if(!pDevice->pDevRxBuffer || !pDevice->pDevTxBuffer)
	{
		eRtnCode = Zi_GENERAL_ERROR;
		goto CHIP_TEST_RET;
	}

	eRtnCode = Zi_GetChipVersion(pDevice);
	if(Zi_RTN_OK == eRtnCode)
	{
		cmmb_chip_type = CMMB_ZM2167_DEVICE;
	}
	else
	{
		cmmb_chip_type = CMMB_SMS1186_DEVICE;
	}

CHIP_TEST_RET:
	if(pDevice->pDevRxBuffer)
	{
		Zi_free(pDevice->pDevRxBuffer);
		pDevice->pDevRxBuffer = NULL;
	}

	if(pDevice->pDevTxBuffer)
	{
		Zi_free(pDevice->pDevTxBuffer);
		pDevice->pDevTxBuffer = NULL;
	}

	if(pDevice)
	{
		Zi_free(pDevice);
		pDevice = NULL; 
	}

	return eRtnCode;	
}
///////////////////////////////////////////////////////////////////////////////
Zi_HANDLE Zi_OpenDevice(Zi_VOID)
{
	Zi_RETURN_CODE eRtnCode = Zi_GENERAL_ERROR;
	Zi_DEVICE_HANDLE *pDevice = NULL;

	Zi_CloseDevice(Zi_pCmmbDevice);

	pDevice = (Zi_DEVICE_HANDLE *)Zi_alloc(sizeof(Zi_DEVICE_HANDLE));
	if(!pDevice)
	{
		goto INIT_DEVICE_ERR;
	}
	Zi_memset(pDevice, 0, sizeof(Zi_DEVICE_HANDLE));

	Zi_pCmmbDevice = pDevice;
	
	pDevice->tDeviceConfig = Zi_DeviceConfig;
	
	Zi_log("________Open Device, Begin________\n");
	Zi_log("CMMB SDK version-%s\n", Zi_CMMB_SDK_VERSION);

   	// Configure slave spi
	pDevice->pDevRxBuffer = (Zi_UINT8 *)kmalloc(SPI_BURST_LEN + 256, GFP_KERNEL | __GFP_DMA);
	pDevice->pDevTxBuffer = (Zi_UINT8 *)kmalloc(SPI_BURST_LEN + 256, GFP_KERNEL | __GFP_DMA);
	if(!pDevice->pDevRxBuffer || !pDevice->pDevTxBuffer)
	{
		goto INIT_DEVICE_ERR;
	}

	eRtnCode = Zi_SetupSlaveSpi(pDevice);
	if( Zi_RTN_OK != eRtnCode)
	{
		Zi_log("Setup cmmb chip spi fail!!!\n");
		goto INIT_DEVICE_ERR;
	}
	// Set TS disable
	Zi_WriteRegFields(pDevice, REG_MP2TS_CTRL, 5, 1, 0x0);	
	// Interrupt polarity selection: 0 - Low is active; 1 - High is active
	Zi_WriteRegFields(pDevice, REG_SDIO_I2C_CTRL, 2, 1, pDevice->tDeviceConfig.eIntLevel);
	// Interrupt control signal: when use SPI/SDIO, it must be set to '0'
	Zi_WriteRegFields(pDevice, REG_SDIO_I2C_CTRL, 3, 1, pDevice->tDeviceConfig.nIntType);   
    
	Zi_WriteReg(pDevice, REG_FM_CMMB_SEL, 0x00);

#if 0
	// Get chip version
	eRtnCode = Zi_GetChipVersion(pDevice);
	if(Zi_RTN_OK != eRtnCode)
	{
		Zi_log("Get chip version fail!!!\n");
		goto INIT_DEVICE_ERR;
	}
#endif

	eRtnCode = Zi_WriteCoeff(pDevice);
	if(Zi_RTN_OK != eRtnCode)
	{
		Zi_log("Write coeff fail!!!\n");
		goto INIT_DEVICE_ERR;
	}

	// Init Tunner
	eRtnCode = Zi_InitTuner(pDevice);
	if(Zi_RTN_OK != eRtnCode)
	{
		Zi_log("Init tuner fail!!!\n");
		goto INIT_DEVICE_ERR;
	}

#if 0
	// Init 7816
	eRtnCode = Zi_InitScc(pDevice);
	if( Zi_RTN_OK != eRtnCode ) 
	{
		Zi_log("Init scc fail!!!\n");
		goto INIT_DEVICE_ERR;
    }
#endif

	Zi_WriteRegFields(pDevice, REG_SC0_CONFIG_B, 0, 5, 0x00);
	Zi_WriteRegFields(pDevice, REG_SC0_CONFIG_A, 0, 5, 0x00);
	Zi_WriteRegFields(pDevice, REG_SC1_CONFIG_B, 0, 5, 0x00);
	Zi_WriteRegFields(pDevice, REG_SC1_CONFIG_A, 0, 5, 0x00);
	Zi_WriteRegFields(pDevice, REG_SC2_CONFIG_B, 0, 5, 0x00);
	Zi_WriteRegFields(pDevice, REG_SC2_CONFIG_A, 0, 5, 0x00);
	Zi_WriteRegFields(pDevice, REG_SC3_CONFIG_B, 0, 5, 0x00);
	Zi_WriteRegFields(pDevice, REG_SC3_CONFIG_A, 0, 5, 0x00);
	Zi_WriteRegFields(pDevice, REG_SC4_CONFIG_B, 0, 5, 0x00);
	Zi_WriteRegFields(pDevice, REG_SC4_CONFIG_A, 0, 5, 0x00);
	Zi_WriteReg(pDevice, REG_SC_VALID_MISC, 0x00);

	Zi_WriteReg(pDevice, REG_HIC1_INT_CLEAR, 0x3f);
	Zi_WriteReg(pDevice, REG_SCC_INT_CLEAR, 0x0f);

	Zi_WriteReg(pDevice, REG_CTRL, 0x00); // enable IQ,DAGC,RFO,IFO,FFO SFO function

	Zi_WriteReg(pDevice, REG_FTS_MISC_DENO,0x6d);  // set FTS deno
	Zi_WriteReg(pDevice, REG_FTS_MISC_RATIO,0x88);  // set FTS threshold
	Zi_WriteReg(pDevice, REG_OUT_SEL,0x05);  //output to qam demap/tsd/bydi head bit sel
	Zi_WriteReg(pDevice, REG_AUTOSCALE_CONFIG,0x93);  // feedfoward enable,feedback disable
	//Zi_WriteReg(pDevice, REG_DCR_MISC,0x5b);  //  remove DC and DP
	Zi_WriteReg(pDevice, REG_DCR_MISC,0x5a);
	Zi_WriteReg(pDevice, REG_STC_MISC,0x95);  // notch enable and notch width set

	Zi_WriteReg(pDevice, REG_SFO_MAX_SWITCH_TH,0x70); // the threshold of max echo switch
	Zi_WriteReg(pDevice, REG_SFO_INIT_MISC,0x72); // SFO work slot num before TSD and mini step of init SFO
	Zi_WriteReg(pDevice, REG_SFO_MISC,0x0f); // SFO track mode and  mini step in normal stage
	Zi_WriteReg(pDevice, REG_LDPC_ITER_NUM_2QAM, 0x13); // BPSK max iter nbum
	Zi_WriteReg(pDevice, REG_LDPC_ITER_NUM_4QAM, 0x13); // QPSK max iter nbum
	Zi_WriteReg(pDevice, REG_LDPC_ITER_NUM_16QAM, 0x0e); // 16QAM max iter nbum
	Zi_WriteReg(pDevice, REG_MISC_CTRL, 0xfe); // enable clock gating;
	Zi_WriteReg(pDevice, REG_ADC_CONFIG, 0x08);  // ADC always on
	Zi_WriteReg(pDevice, REG_STC_PAR_TH, 0x42);  // STC thld  on REG_STC_PAR_TH
	//Zi_WriteReg(pDevice, REG_AGC_IQREF, 0x42);  // IQ reference
	Zi_WriteReg(pDevice, REG_AGC_IQREF, 0x60);
	Zi_WriteReg(pDevice, REG_AGC_NW, 0x20);
	Zi_WriteReg(pDevice, REG_RFO_DELAY_CFG, 0x11);
	Zi_WriteReg(pDevice, REG_AGC_PARA_N, 0x04);

	pDevice->Zi_SysStatus = (Zi_SYS_STATUS *)Zi_alloc(sizeof(Zi_SYS_STATUS));
	if(!pDevice->Zi_SysStatus)
	{
		goto INIT_DEVICE_ERR;
	}
	Zi_memset(pDevice->Zi_SysStatus, 0, sizeof(Zi_SYS_STATUS));

	pDevice->bCLCHEnable = FALSE;
	Zi_cmmb_state = 0;

	eRtnCode = DMX_Initialize(pDevice);
	if(Zi_RTN_OK != eRtnCode)
	{
		goto INIT_DEVICE_ERR;
	}
	
	Zi_EnableInt(pDevice, 1);

	Zi_log("________End________\n");
	
	return pDevice;
	
INIT_DEVICE_ERR:
	Zi_CloseDevice(pDevice);
	return NULL;
}

///////////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_CmdSetFreq(Zi_UINT32 freqpoint)
{
	Zi_RETURN_CODE eRtnCode = Zi_GENERAL_ERROR;
	
	eRtnCode = Zi_LockFrequency(Zi_pCmmbDevice, freqpoint, 500);
	
	Zi_CmmbCmdData[0]++;
	Zi_CmmbCmdData[7] = Zi_FRAME_TYPE_COMMAND_RESPONSE;
	Zi_CmmbCmdData[8] = Zi_CTRL_CMD_TYPE_SET_FREQ;
	Zi_CmmbCmdData[9] = eRtnCode;
	
	smschar_interrupt();

	return eRtnCode;
}

///////////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_Cmmb_Select_Service(Zi_UINT16 serviceId)
{
	Zi_RETURN_CODE eRtnCode = Zi_GENERAL_ERROR;
	Zi_UINT32 channelNum = 0xff;
	
	Zi_log("________Open service: serviceId = %d________", serviceId);

	if(0 == serviceId)
	{
		eRtnCode = Zi_OpenCLCHService(Zi_pCmmbDevice);
	}
	else
	{
		eRtnCode = Zi_PlayService(Zi_pCmmbDevice, serviceId, &channelNum);
		
	}

	
	Zi_CmmbCmdData[0]++;
	Zi_CmmbCmdData[7] = Zi_FRAME_TYPE_COMMAND_RESPONSE;
	Zi_CmmbCmdData[8] = Zi_CTRL_CMD_TYPE_SET_LOGIC_CHANNEL;
	Zi_CmmbCmdData[9] = eRtnCode;
	Zi_CmmbCmdData[10] = serviceId & 0xff;
	Zi_CmmbCmdData[11] = (serviceId >> 8) & 0xff;
	if(channelNum != 0xff)
	{
		Zi_CmmbCmdData[12] = channelNum + 2;
	}
		
	smschar_interrupt();
	
	return eRtnCode;
}

///////////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_Cmmb_Stop_Service(Zi_UINT16 serviceId)
{
	Zi_RETURN_CODE eRtnCode = Zi_GENERAL_ERROR;

	Zi_log("________Stop service: serviceId = %d________\n", serviceId);
	
	if(0 == serviceId)
	{
		eRtnCode = Zi_StopCLCHService(Zi_pCmmbDevice);
	}
	else
	{	
		eRtnCode = Zi_StopService(Zi_pCmmbDevice, serviceId);	
	}
	
	Zi_CmmbCmdData[0]++;
	Zi_CmmbCmdData[7] = Zi_FRAME_TYPE_COMMAND_RESPONSE;
	Zi_CmmbCmdData[8] = Zi_CTRL_CMD_TYPE_CLOSE_LOGIC_CHANNEL;
	Zi_CmmbCmdData[9] = eRtnCode;
	Zi_CmmbCmdData[10] = serviceId & 0xff;
	Zi_CmmbCmdData[11] = (serviceId >> 8) & 0xff;

	smschar_interrupt();
	
	return eRtnCode;
}

///////////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_CmdISO7816Reset(Zi_VOID)
{
	Zi_RETURN_CODE eRtnCode = Zi_GENERAL_ERROR;
	Zi_UINT32 buflen;
	
	Zi_log("ISO7816 reset\n");
	
	eRtnCode = Zi_InitScc(Zi_pCmmbDevice, Zi_dev_area + 4*64*1024, &buflen);
	if( Zi_RTN_OK != eRtnCode ) 
	{
		Zi_log("Init scc fail!!!\n");
	}

	Zi_Cmmb7816Data[0]++;
	Zi_Cmmb7816Data[4] = buflen & 0xff;
	Zi_Cmmb7816Data[5] = (buflen >> 8) & 0xff;
	Zi_Cmmb7816Data[6] = (buflen >> 16) & 0xff;
	Zi_Cmmb7816Data[7] = Zi_FRAME_TYPE_7816_NCMD_RESPONSE;
	Zi_Cmmb7816Data[8] = Zi_7816_TYPE_RESET;
	Zi_Cmmb7816Data[9] = eRtnCode;

    	smschar_interrupt();

	return eRtnCode;
}

///////////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE Zi_CmdAPDU(Zi_UINT8 *APDUCmd, Zi_UINT32 CmdLength)
{
	return Zi_SccSendComand(Zi_pCmmbDevice, APDUCmd, CmdLength);
}

///////////////////////////////////////////////////////////////////////////////
Zi_VOID Zi_CmdProcess(Zi_HANDLE hDevice)
{
	Zi_RETURN_CODE  eRtnCode;
	Zi_CMMB_INT_TYPE eResult;
	Zi_UINT32 bufLen = 0;
	Zi_UINT8 channelNum;
	Zi_UINT16  sw;
	Zi_UINT8 sccType = 0xFF;
	Zi_UINT16 serviceId;
	Zi_DEVICE_HANDLE *pDevice = (Zi_DEVICE_HANDLE *)hDevice;

	Zi_cmmb_state |= CMMB_INT_PROCESS_FLAG;

	if ((Zi_cmmb_state & CMMB_DEVICE_RELEASE_FLAG) || NULL == pDevice)
	{
		Zi_cmmb_state &= ~CMMB_INT_PROCESS_FLAG;
		return;
	}
	
	Zi_CmdData.DataLen = 0;
	Zi_CmdData.ChannelNumber = 0;

	eResult = Zi_GetCurState(pDevice, &bufLen, &channelNum);
	Zi_CmdData.ChannelNumber = channelNum;
	if(eResult < 0)
	{
		Zi_log("Unsuport interrupt happen!!!\n");
	}
	else
	{
		switch(eResult)
		{
			case Zi_CMMB_TIMEOUT_INTERRUPT:
				Zi_Wait_ReLockFreq(pDevice, 500);
				break;
			case Zi_CMMB_SC_DAT_RDY:

				eRtnCode = Zi_GetServiceData(pDevice, Zi_dev_area+DataCount*64*1024, bufLen, channelNum);

				if(Zi_RTN_OK != eRtnCode)
				{
					Zi_log("Service data err!!!\n");
					break;
				}
#if 0				
				if(FALSE == Zi_IsMpxFrameHeader(Zi_dev_area+DataCount*64*1024, bufLen, &Zi_CmdData.DataLen))
				{
					Zi_log("MPX FrameHeader err!!!\n");
					break;
				}
#else
				Zi_CmdData.DataLen = bufLen;
#endif

				if(5 == channelNum)
				{					
					
					if(CMMB_GETTS0_PARSER_FLAG != (Zi_cmmb_state & CMMB_GETTS0_PARSER_FLAG))
					{
						Zi_CmdData.Data = Zi_dev_area+DataCount*64*1024;
						Zi_CLCHDataParser(pDevice, &Zi_CmdData);
					}
					else
					{
						if(!pDevice->bCLCHEnable)
							break;						
					}
				}
			       
				serviceId = pDevice->ActiveService[channelNum].nActiveServiceID;
				if(0 == serviceId)
				{
					Zi_log("Service had closed!!!\n");
					break;
				}
				serviceId = (serviceId == 0xffff ? 0 : serviceId);
				Zi_log("MFdata datalen = 0x%x, serviceId = %d\n", bufLen, serviceId);
				
				Zi_CmmbFrameHeadrData[0]++; 
				Zi_CmmbFrameHeadrData[4] = Zi_CmdData.DataLen & 0xff;
				Zi_CmmbFrameHeadrData[5] = (Zi_CmdData.DataLen>>8) & 0xff;
				Zi_CmmbFrameHeadrData[6] = (Zi_CmdData.DataLen>>16) & 0xff;
				Zi_CmmbFrameHeadrData[7] = Zi_FRAME_TYPE_MULTIPLEX_FRAME_DATA;
				Zi_CmmbFrameHeadrData[10] = DataCount;
				Zi_CmmbFrameHeadrData[11] = serviceId & 0xff;
				Zi_CmmbFrameHeadrData[12] = (serviceId >> 8) & 0xff;
				if(channelNum == 5)
				{
					Zi_CmmbFrameHeadrData[13] = 1;
				}
				else if(channelNum <= 4 && channelNum >= 0)
				{
					Zi_CmmbFrameHeadrData[13] = channelNum + 2;
				}
				
				if(DataCount==3)
				{
					DataCount=0;
				}
				else
				{
					DataCount++;
				}

				smschar_interrupt();		    
				break;
			case Zi_CMMB_SCC_TRANS_INT:
				eRtnCode = Zi_SccGetResponse(pDevice, g_7816cmd, g_7816bufLen, Zi_dev_area+4*64*1024, &Zi_CmdData.DataLen, &sw , &sccType);
				Zi_Cmmb7816Data[0]++;
				Zi_Cmmb7816Data[4] = Zi_CmdData.DataLen & 0xff;
				Zi_Cmmb7816Data[5] = (Zi_CmdData.DataLen>>8) & 0xff;
				Zi_Cmmb7816Data[6] = (Zi_CmdData.DataLen>>16) & 0xff;
				Zi_Cmmb7816Data[7] = Zi_FRAME_TYPE_7816_NCMD_RESPONSE;
				Zi_Cmmb7816Data[8] = sccType;
				Zi_Cmmb7816Data[9] = eRtnCode;
				Zi_Cmmb7816Data[10] = (sw >> 8) & 0xFF;
				Zi_Cmmb7816Data[11] = sw & 0xFF;
				smschar_interrupt();   
				break;
			default:
				break;
		}
	}
	Zi_EnableInt(pDevice, 1);
	Zi_cmmb_state &= ~CMMB_INT_PROCESS_FLAG;
	Zi_irq_level(3);
}
///////////////////////////////////////////////////////////////////////////////
Zi_VOID Zi_CLCHDataParser(Zi_HANDLE hDevice, Zi_CMD_DATA_T *CmdData)
{
	Zi_UINT8 *MFData;
	Zi_UINT32 MFDataLen;
	Zi_UINT32 MSFNum;
	Zi_UINT32 ListLen[15];
	Zi_UINT32 Temp;
	Zi_UINT32 MFNum_Cmct;
	Zi_UINT32 MSFNum_Cmct;;
	Zi_UINT32 TsNum_Cmct;
	Zi_UINT8 FrameID_Cmct;
	Zi_UINT8 ModulateType_Cmct;
	Zi_UINT8 LDPCRate_Cmct;
	Zi_UINT8 RsDecodeType_Cmct;
	Zi_UINT8 ByteInterleaveMode_Cmct;
	Zi_UINT8 ScramblingMode;
	Zi_UINT8 TsCount_Cmct;
	Zi_UINT8 TsStartNumber_Cmct;
	Zi_UINT16 ServiceID_Cmct[15];
	Zi_CHAR TempStr[16];
	Zi_CHAR ServIdList[50][48] ={0};
	Zi_UINT32 ServiceIdListNum = 0;
	Zi_UINT32 i, j, n;
	static Zi_UINT32 k = 0;
	Zi_DEVICE_HANDLE *pDevice = (Zi_DEVICE_HANDLE *)hDevice;

	if (NULL == pDevice || !pDevice->pDemuxerPrivateStruct)
	{
		return;
	}
	Zi_DEMUXER_CONTEXT *pContext = (Zi_DEMUXER_CONTEXT *)pDevice->pDemuxerPrivateStruct;
	
	MFData = CmdData->Data;
	//MF head
	if(0 != Zi_memcmp(MFData, MF_FRAME_START_TAG, 4))
	{
		for(i = 0; i < 4; i++)
		{
			Zi_log("MFData[%d] = %d\n", i, MFData[i]);
		}
		Zi_log("Can't find frame start tag[%s]!!!\n", MF_FRAME_START_TAG);
		return;
	}
	
	MFDataLen = 12;
	MSFNum = *(MFData + 11) & 0x0f;
	for(i = 0; i < MSFNum; i++)
	{
		ListLen[i] = 0;
		ListLen[i] = *(MFData+MFDataLen+2) | (*(MFData+MFDataLen+1)<<8) | (*(MFData+MFDataLen)<<16);
		MFDataLen += 3;
	}
	Temp = *(MFData + 7) & 0x20;
	if(Temp == 0x20)
	{
		MFDataLen += 5;
	}
	MFDataLen += 4;
	if(MFDataLen > CmdData->DataLen)
	{
		Zi_log("MF data length err!!!\n");
		return;
	}
	MFData += MFDataLen;
	
	//List
	for(i = 0; i < MSFNum; i++)
	{
		if((MFDataLen+ListLen[i]) > CmdData->DataLen)
		{
			Zi_log("(MFDataLen+ListLen[i]) = %d, CmdData->DataLen = %d\n", MFDataLen+ListLen[i], CmdData->DataLen);
			return;
		}
		if(*MFData == 0x02)
		{
			MFNum_Cmct  = *(MFData + 3) & 0x3f;
			MFData += 4;

			pContext->nServiceCount = MFNum_Cmct;
			pContext->pSeviceChannelList = (Zi_CMMBSERVICE_INFO *) Zi_alloc( sizeof(Zi_CMMBSERVICE_INFO)*(pContext->nServiceCount));
			if(!pContext->pSeviceChannelList)
			{
				return;
			}
			Zi_memset(pContext->pSeviceChannelList, 0, sizeof(Zi_CMMBSERVICE_INFO)*(pContext->nServiceCount));
			
			for(j = 0; j < MFNum_Cmct; j++)
			{
				FrameID_Cmct = ((*(MFData) & 0xfc) >> 2);
				RsDecodeType_Cmct = *(MFData) & 0x03;
				ModulateType_Cmct = ((*(MFData+1) & 0x0c) >> 2);
				LDPCRate_Cmct = ((*(MFData+1) & 0x30) >> 4);
				ByteInterleaveMode_Cmct = ((*(MFData+1) & 0xc0) >> 6);
				ScramblingMode   = ((*(MFData+1)&0x01)<<2) + ((*(MFData+2)&0xc0)>>6);
				TsCount_Cmct = (*(MFData+2) & 0x2f);
				TsStartNumber_Cmct = ((*(MFData+3) & 0xfc) >> 2);

				TsNum_Cmct = *(MFData + 2) & 0x3f;
				MFData += 3+TsNum_Cmct;

				MSFNum_Cmct = *(MFData) & 0x0f;
				MFData += 1;
				for(n = 0; n < MSFNum_Cmct; n++ )
				{
					ServiceID_Cmct[n] = *(MFData+2) | (*(MFData+1)<<8);
					MFData += 3;
				}
				if(0 != FrameID_Cmct)
				{
						sprintf(ServIdList[k], "%d %d %d %d %d %d %d", FrameID_Cmct, RsDecodeType_Cmct, ByteInterleaveMode_Cmct, 
										LDPCRate_Cmct, ModulateType_Cmct, TsCount_Cmct, TsStartNumber_Cmct);
						for(n = 0; n < MSFNum_Cmct; n++)
						{
							Zi_memset(TempStr, 0x00, 16);
							sprintf(TempStr, " %d", ServiceID_Cmct[n]);
							pContext->pSeviceChannelList[ServiceIdListNum].FrameID_Cmct = FrameID_Cmct;
							pContext->pSeviceChannelList[ServiceIdListNum].RsDecodeType_Cmct = RsDecodeType_Cmct;
							pContext->pSeviceChannelList[ServiceIdListNum].ByteInterleaveMode_Cmct = ByteInterleaveMode_Cmct;
							pContext->pSeviceChannelList[ServiceIdListNum].LDPCRate_Cmct = LDPCRate_Cmct;
							pContext->pSeviceChannelList[ServiceIdListNum].ModulateType_Cmct = ModulateType_Cmct;
							pContext->pSeviceChannelList[ServiceIdListNum].TsCount_Cmct = TsCount_Cmct;
							pContext->pSeviceChannelList[ServiceIdListNum].TsStartNumber_Cmct = TsStartNumber_Cmct;
							pContext->pSeviceChannelList[ServiceIdListNum].ServiceID_Cmct = ServiceID_Cmct[n];
							ServiceIdListNum++;
							
							Zi_strcat(ServIdList[k], TempStr);
						}
						Zi_log("ServIdList[%d] = [%s]\n", k, ServIdList[k]);
						k++;
						if(50 == k)
						{
							k = 0;
						} 	
				}
			}
			Zi_cmmb_state |= CMMB_GETTS0_PARSER_FLAG;
			pContext->nServiceCount=ServiceIdListNum;
			break;
		}
		else
		{
			MFData += ListLen[i];
			MFDataLen += ListLen[i];
		}
	}
}

Zi_BOOL Zi_IsMpxFrameHeader( Zi_UINT8 *pBuf, Zi_UINT32 BufSize, Zi_UINT32 *pOutFrameSize )
{
	Zi_UINT32 HeaderLengthInclCrc;
	Zi_UINT32 NumSubFrames;
	Zi_UINT32 Offset;

	*pOutFrameSize = 0;
	// Check for start code ( 00 00 00 01 )
	if ( BufSize < 11 || pBuf[0] != 0 || pBuf[1] != 0 || pBuf[2] != 0 || pBuf[3] != 1 )
	{
		return FALSE;
	}

	// Get the header length
	HeaderLengthInclCrc = pBuf[4] + 4;
	if ( HeaderLengthInclCrc > BufSize )
	{
		return FALSE;
	}

	// Compute the header CRC
	if (Zi_CRC32Compute(0xFFFFFFFF, pBuf, HeaderLengthInclCrc) != 0 )
	{
		return FALSE;
	}

	// Frame ID is the lower 6 bits of the byte offset 6

	// Calculate the multiplex frame size
	// The size consists of header size + CRC + sizes of all sub frames
	// Sizes of sub frames are stored in an array at offset 11 in the 
	// frame header
	NumSubFrames = pBuf[11]&0xF;
	Offset = 12;
	// Add up all the sub frames' lengths
	while( Offset < BufSize && NumSubFrames > 0 )
	{
		*pOutFrameSize += pBuf[Offset] << 16;
		Offset++;
		*pOutFrameSize += pBuf[Offset] << 8;
		Offset++;
		*pOutFrameSize += pBuf[Offset];
		Offset++;
		NumSubFrames--;
	}
	*pOutFrameSize += HeaderLengthInclCrc;

	return TRUE;
}

Zi_UINT32 Zi_CRC32Compute(Zi_UINT32 InitialVector, Zi_UINT8 *pData, Zi_UINT32 Len)
{
	Zi_UINT32 s_CrcTable[] = 
	{
		0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9, 0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
		0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61, 0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
		0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
		0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
		0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039, 0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
		0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
		0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
		0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1, 0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
		0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
		0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,
		0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde, 0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
		0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
		0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
		0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6, 0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
		0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
		0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,
		0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637, 0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
		0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
		0x251d3b9e, 0x21dc2629,	0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
		0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff, 0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
		0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
		0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
		0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7, 0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
		0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
		0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
		0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8, 0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
		0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30, 0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
		0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
		0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0, 0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
		0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
		0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
		0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668, 0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
	};
	Zi_INT32	i;
	Zi_INT32	j;
	Zi_UINT32 crc = InitialVector;

	for (j = 0; (j < Len);  j++)
	{
		i = ((crc >> 24) ^ *pData++) & 0xff;
		crc = (crc << 8) ^ s_CrcTable[i];
	}
	return crc;
}

///////////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE DMX_Initialize(Zi_HANDLE hDevice)
{
	Zi_DEMUXER_CONTEXT *pDemuxerContext = (Zi_DEMUXER_CONTEXT *) Zi_alloc(sizeof(Zi_DEMUXER_CONTEXT));
	if(NULL == hDevice || NULL == pDemuxerContext)
	{
		return Zi_GENERAL_ERROR;
	}
	Zi_memset(pDemuxerContext, 0x00, sizeof(Zi_DEMUXER_CONTEXT));

	((Zi_DEVICE_HANDLE *)hDevice)->pDemuxerPrivateStruct = pDemuxerContext;

	return Zi_RTN_OK;
}

///////////////////////////////////////////////////////////////////////////////
Zi_RETURN_CODE DMX_Destory(Zi_HANDLE hDevice)
{
	if (NULL == hDevice)
	{
		return Zi_GENERAL_ERROR;
	}
	if(((Zi_DEVICE_HANDLE*)hDevice)->pDemuxerPrivateStruct)
	{
		if(((Zi_DEMUXER_CONTEXT *)(((Zi_DEVICE_HANDLE *)hDevice)->pDemuxerPrivateStruct))->pSeviceChannelList)
		{
			Zi_free(((Zi_DEMUXER_CONTEXT *)(((Zi_DEVICE_HANDLE *)hDevice)->pDemuxerPrivateStruct))->pSeviceChannelList);
			((Zi_DEMUXER_CONTEXT *)(((Zi_DEVICE_HANDLE *)hDevice)->pDemuxerPrivateStruct))->pSeviceChannelList = NULL;
		}
		Zi_free(((Zi_DEVICE_HANDLE *)hDevice)->pDemuxerPrivateStruct);
		((Zi_DEVICE_HANDLE *)hDevice)->pDemuxerPrivateStruct = NULL;
	}
	return Zi_RTN_OK;
}

///////////////////////////////////////////////////////////////////////////////

Zi_RETURN_CODE Zi_CloseDevice(Zi_HANDLE hDevice)
{
	Zi_DEVICE_HANDLE *pDevice = (Zi_DEVICE_HANDLE *) hDevice;

	if( NULL == pDevice )
	{
		return Zi_RTN_OK;
	}

	Zi_cmmb_state |=  CMMB_DEVICE_RELEASE_FLAG;

	while(CMMB_INT_PROCESS_FLAG == (Zi_cmmb_state & CMMB_INT_PROCESS_FLAG))
	{
		Zi_msleep(100);
	}

	Zi_EnableInt(pDevice, 0);
	
	Zi_WriteReg(pDevice, REG_SC_VALID_MISC, 0x00);
	
	Zi_ResetDevice(pDevice);
	
	Zi_StopSlaveSpi(pDevice);
	
	DMX_Destory(pDevice);

	if (pDevice->pDevRxBuffer)
	{
		Zi_free(pDevice->pDevRxBuffer);
		pDevice->pDevRxBuffer = NULL;
	}
	if (pDevice->pDevTxBuffer)
	{
		Zi_free(pDevice->pDevTxBuffer);
		pDevice->pDevTxBuffer = NULL;
	}
	
	if(pDevice->Zi_SysStatus)
	{
		Zi_free(pDevice->Zi_SysStatus);
		pDevice->Zi_SysStatus = NULL;
	}
	
	if (pDevice)
	{
		Zi_free(pDevice);
		pDevice = NULL;
	}
	
	Zi_pCmmbDevice = NULL;
	
	return Zi_RTN_OK;
}


