/**********************************************************************************/
/*  Copyright (c) 2011, Silicon Image, Inc.  All rights reserved.                 */
/*  No part of this work may be reproduced, modified, distributed, transmitted,   */
/*  transcribed, or translated into any language or computer format, in any form  */
/*  or by any means without written permission of: Silicon Image, Inc.,           */
/*  1140 East Arques Avenue, Sunnyvale, California 94085                          */
/**********************************************************************************/

//------------------------------------------------------------------------------
// Driver API typedefs
//------------------------------------------------------------------------------
//
// structure to hold command details from upper layer to CBUS module
//
typedef struct
{
    uint8_t reqStatus;       // CBUS_IDLE, CBUS_PENDING
    uint8_t retryCount;
    uint8_t command;         // VS_CMD or RCP opcode
    uint8_t offsetData;      // Offset of register on CBUS or RCP data
    uint8_t length;          // Only applicable to write burst. ignored otherwise.
    union
    {
    uint8_t msgData[ 16 ];   // Pointer to message data area.
	unsigned char	*pdatabytes;			// pointer for write burst or read many bytes
    }payload_u;

} cbus_req_t;


//
// Functions that driver exposes to the upper layer.
//
//////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxChipInitialize
//
// Chip specific initialization.
// This function is for SiI 9244 Initialization: HW Reset, Interrupt enable.
//
//
//////////////////////////////////////////////////////////////////////////////
bool_t 	SiiMhlTxChipInitialize( void );


//////////////////////////////////////////////////////////////////////////////
//
// SiiMhlSetEnableStatus
//
// Chip Specific enable/disable of MHL discovery/power management
//
void SiiMhlSetEnableStatus(bool_t enable);


///////////////////////////////////////////////////////////////////////////////
// SiiMhlTxDeviceIsr
//
// This function must be called from a master interrupt handler or any polling
// loop in the host software if during initialization call the parameter
// interruptDriven was set to true. SiiMhlTxGetEvents will not look at these
// events assuming firmware is operating in interrupt driven mode. MhlTx component
// performs a check of all its internal status registers to see if a hardware event
// such as connection or disconnection has happened or an RCP message has been
// received from the connected device. Due to the interruptDriven being true,
// MhlTx code will ensure concurrency by asking the host software and hardware to
// disable interrupts and restore when completed. Device interrupts are cleared by
// the MhlTx component before returning back to the caller. Any handling of
// programmable interrupt controller logic if present in the host will have to
// be done by the caller after this function returns back.

// This function has no parameters and returns nothing.
//
// This is the master interrupt handler for 9244. It calls sub handlers
// of interest. Still couple of status would be required to be picked up
// in the monitoring routine (Sii9244TimerIsr)
//
// To react in least amount of time hook up this ISR to processor's
// interrupt mechanism.
//
// Just in case environment does not provide this, set a flag so we
// call this from our monitor (Sii9244TimerIsr) in periodic fashion.
//
// Device Interrupts we would look at
//		RGND		= to wake up from D3
//		MHL_EST 	= connection establishment
//		CBUS_LOCKOUT= Service USB switch
//		RSEN_LOW	= Disconnection deglitcher
//		CBUS 		= responder to peer messages
//					  Especially for DCAP etc time based events
//
void 	SiiMhlTxDeviceIsr( void );

///////////////////////////////////////////////////////////////////////////////
// SiiMhlTxDrvSendCbusCommand
//
// Write the specified Sideband Channel command to the CBUS.
// Command can be a MSC_MSG command (RCP/RAP/RCPK/RCPE/RAPK), or another command 
// such as READ_DEVCAP, SET_INT, WRITE_STAT, etc.
//
// Parameters:  
//              pReq    - Pointer to a cbus_req_t structure containing the 
//                        command to write
// Returns:     true    - successful write
//              false   - write failed
///////////////////////////////////////////////////////////////////////////////
bool_t	SiiMhlTxDrvSendCbusCommand ( cbus_req_t *pReq  );

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvPowBitChange
//
// Alert the driver that the peer's POW bit has changed so that it can take 
// action if necessary.
//
void	SiiMhlTxDrvPowBitChange( bool_t enable );

///////////////////////////////////////////////////////////////////////////////
//
// Function Name: MHLSinkOrDonglePowerStatusCheck()
//
// Function Description: Check MHL device is dongle or sink to set inputting current limitation.
//
void MHLSinkOrDonglePowerStatusCheck (void);


///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDriverTmdsControl
//
// Control the TMDS output. MhlTx uses this to support RAP content on and off.
//
void	SiiMhlTxDrvTmdsControl( bool_t enable );

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvNotifyEdidChange
//
// MhlTx may need to inform upstream device of a EDID change. This can be
// achieved by toggling the HDMI HPD signal or by simply calling EDID read
// function.
//
void	SiiMhlTxDrvNotifyEdidChange ( void );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxReadDevcap
//
// This function sends a READ DEVCAP MHL command to the peer.
// It  returns true if successful in doing so.
//
// The value of devcap should be obtained by making a call to SiiMhlTxGetEvents()
//
// offset		Which byte in devcap register is required to be read. 0..0x0E
//
bool_t SiiMhlTxReadDevcap( uint8_t offset );

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvGetScratchPad
//
// This function reads the local scratchpad into a local memory buffer
//
void SiiMhlTxDrvGetScratchPad(uint8_t startReg,uint8_t *pData,uint8_t length);

//
// Functions that driver expects from the upper layer.
//

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxNotifyDsHpdChange
//
// Informs MhlTx component of a Downstream HPD change (when h/w receives
// SET_HPD or CLEAR_HPD).
//
extern	void	SiiMhlTxNotifyDsHpdChange( uint8_t dsHpdStatus );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxNotifyConnection
//
// This function is called by the driver to inform of connection status change.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
extern	void	SiiMhlTxNotifyConnection( bool_t mhlConnected );

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxNotifyMhlEnabledStatusChange
//
//  This function is invoked from the MhlTx driver to notify the Component layer
//  about changes to the Enabled state of the MHL subsystem.
//
// Application module must provide this function.
//
void  SiiMhlTxNotifyMhlEnabledStatusChange(bool_t enabled);
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxMscCommandDone
//
// This function is called by the driver to inform of completion of last command.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
extern	void	SiiMhlTxMscCommandDone( uint8_t data1 );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlIntr
//
// This function is called by the driver to inform of arrival of a MHL INTERRUPT.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
extern	void	SiiMhlTxGotMhlIntr( uint8_t intr_0, uint8_t intr_1 );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlStatus
//
// This function is called by the driver to inform of arrival of a MHL STATUS.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
extern	void	SiiMhlTxGotMhlStatus( uint8_t status_0, uint8_t status_1 );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlMscMsg
//
// This function is called by the driver to inform of arrival of a MHL STATUS.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
// Application shall not call this function.
//
extern	void	SiiMhlTxGotMhlMscMsg( uint8_t subCommand, uint8_t cmdData );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlWriteBurst
//
// This function is called by the driver to inform of arrival of a scratchpad data.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
// Application shall not call this function.
//
extern	void	SiiMhlTxGotMhlWriteBurst( uint8_t *spadArray );

bool_t SiiMhlTxDrvCBusBusy (void);

///////////////////////////////////////////////////////////////////////////////
//
// CBUS register defintions
//
#define REG_CBUS_INTR_STATUS            (TX_PAGE_CBUS | 0x0008)
#define BIT_DDC_ABORT                   (BIT2)    /* Responder aborted DDC command at translation layer */
#define BIT_MSC_MSG_RCV                 (BIT3)    /* Responder sent a VS_MSG packet (response data or command.) */
#define BIT_MSC_XFR_DONE                (BIT4)    /* Responder sent ACK packet (not VS_MSG) */
#define BIT_MSC_XFR_ABORT               (BIT5)    /* Command send aborted on TX side */
#define BIT_MSC_ABORT                   (BIT6)    /* Responder aborted MSC command at translation layer */

#define REG_CBUS_INTR_ENABLE            (TX_PAGE_CBUS | 0x0009)

#define REG_DDC_ABORT_REASON        	(TX_PAGE_CBUS | 0x000C)
#define REG_CBUS_BUS_STATUS             (TX_PAGE_CBUS | 0x000A)
#define BIT_BUS_CONNECTED                   0x01
#define BIT_LA_VAL_CHG                      0x02

#define REG_PRI_XFR_ABORT_REASON        (TX_PAGE_CBUS | 0x000D)

#define REG_CBUS_PRI_FWR_ABORT_REASON   (TX_PAGE_CBUS | 0x000E)
#define	CBUSABORT_BIT_REQ_MAXFAIL			(0x01 << 0)
#define	CBUSABORT_BIT_PROTOCOL_ERROR		(0x01 << 1)
#define	CBUSABORT_BIT_REQ_TIMEOUT			(0x01 << 2)
#define	CBUSABORT_BIT_UNDEFINED_OPCODE		(0x01 << 3)
#define	CBUSSTATUS_BIT_CONNECTED			(0x01 << 6)
#define	CBUSABORT_BIT_PEER_ABORTED			(0x01 << 7)

#define REG_CBUS_PRI_START              (TX_PAGE_CBUS | 0x0012)
#define BIT_TRANSFER_PVT_CMD                0x01
#define BIT_SEND_MSC_MSG                    0x02
#define	MSC_START_BIT_MSC_CMD		        (0x01 << 0)
#define	MSC_START_BIT_VS_CMD		        (0x01 << 1)
#define	MSC_START_BIT_READ_REG		        (0x01 << 2)
#define	MSC_START_BIT_WRITE_REG		        (0x01 << 3)
#define	MSC_START_BIT_WRITE_BURST	        (0x01 << 4)

#define REG_CBUS_PRI_ADDR_CMD           (TX_PAGE_CBUS | 0x0013)
#define REG_CBUS_PRI_WR_DATA_1ST        (TX_PAGE_CBUS | 0x0014)
#define REG_CBUS_PRI_WR_DATA_2ND        (TX_PAGE_CBUS | 0x0015)
#define REG_CBUS_PRI_RD_DATA_1ST        (TX_PAGE_CBUS | 0x0016)
#define REG_CBUS_PRI_RD_DATA_2ND        (TX_PAGE_CBUS | 0x0017)

#define REG_CBUS_PRI_VS_CMD             (TX_PAGE_CBUS | 0x0018)
#define REG_CBUS_PRI_VS_DATA            (TX_PAGE_CBUS | 0x0019)

#define	MSC_REQUESTOR_DONE_NACK         	(0x01 << 6)      

#define	REG_CBUS_MSC_RETRY_INTERVAL		(TX_PAGE_CBUS | 0x001A)		// default is 16
#define	REG_CBUS_DDC_FAIL_LIMIT			(TX_PAGE_CBUS | 0x001C)		// default is 5
#define	REG_CBUS_MSC_FAIL_LIMIT			(TX_PAGE_CBUS | 0x001D)		// default is 5
#define	REG_CBUS_MSC_INT2_STATUS        (TX_PAGE_CBUS | 0x001E)
#define REG_CBUS_MSC_INT2_ENABLE        (TX_PAGE_CBUS | 0x001F)
#define	MSC_INT2_REQ_WRITE_MSC              (0x01 << 0)	// Write REG data written.
#define	MSC_INT2_HEARTBEAT_MAXFAIL          (0x01 << 1)	// Retry threshold exceeded for sending the Heartbeat

#define	REG_MSC_WRITE_BURST_LEN         (TX_PAGE_CBUS | 0x0020)       // only for WRITE_BURST

#define	REG_MSC_HEARTBEAT_CONTROL       (TX_PAGE_CBUS | 0x0021)       // Periodic heart beat. TX sends GET_REV_ID MSC command
#define	MSC_HEARTBEAT_PERIOD_MASK		    0x0F	// bits 3..0
#define	MSC_HEARTBEAT_FAIL_LIMIT_MASK	    0x70	// bits 6..4
#define	MSC_HEARTBEAT_ENABLE			    0x80	// bit 7

#define REG_MSC_TIMEOUT_LIMIT           (TX_PAGE_CBUS | 0x0022)
#define	MSC_TIMEOUT_LIMIT_MSB_MASK	        (0x0F)	        // default is 1
#define	MSC_LEGACY_BIT					    (0x01 << 7)	    // This should be cleared.

#define	REG_CBUS_LINK_CONTROL_1			(TX_PAGE_CBUS | 0x0030)	// 
#define	REG_CBUS_LINK_CONTROL_2			(TX_PAGE_CBUS | 0x0031)	// 
#define	REG_CBUS_LINK_CONTROL_3			(TX_PAGE_CBUS | 0x0032)	// 
#define	REG_CBUS_LINK_CONTROL_4			(TX_PAGE_CBUS | 0x0033)	// 
#define	REG_CBUS_LINK_CONTROL_5			(TX_PAGE_CBUS | 0x0034)	// 
#define	REG_CBUS_LINK_CONTROL_6			(TX_PAGE_CBUS | 0x0035)	// 
#define	REG_CBUS_LINK_CONTROL_7			(TX_PAGE_CBUS | 0x0036)	// 
#define REG_CBUS_LINK_STATUS_1          (TX_PAGE_CBUS | 0x0037)
#define REG_CBUS_LINK_STATUS_2          (TX_PAGE_CBUS | 0x0038)
#define	REG_CBUS_LINK_CONTROL_8			(TX_PAGE_CBUS | 0x0039)	// 
#define	REG_CBUS_LINK_CONTROL_9			(TX_PAGE_CBUS | 0x003A)	// 
#define	REG_CBUS_LINK_CONTROL_10		(TX_PAGE_CBUS | 0x003B)	//
#define	REG_CBUS_LINK_CONTROL_11		(TX_PAGE_CBUS | 0x003C)	// 
#define	REG_CBUS_LINK_CONTROL_12		(TX_PAGE_CBUS | 0x003D)	// 


#define REG_CBUS_LINK_CTRL9_0			(TX_PAGE_CBUS | 0x003A)
#define REG_CBUS_LINK_CTRL9_1           (TX_PAGE_CBUS | 0x00BA)

#define	REG_CBUS_DRV_STRENGTH_0			(TX_PAGE_CBUS | 0x0040)	// 
#define	REG_CBUS_DRV_STRENGTH_1			(TX_PAGE_CBUS | 0x0041)	// 
#define	REG_CBUS_ACK_CONTROL			(TX_PAGE_CBUS | 0x0042)	// 
#define	REG_CBUS_CAL_CONTROL			(TX_PAGE_CBUS | 0x0043)	// Calibration

#define REG_CBUS_SCRATCHPAD_0           (TX_PAGE_CBUS | 0x00C0)
#define REG_CBUS_DEVICE_CAP_0           (TX_PAGE_CBUS | 0x0080)
#define REG_CBUS_DEVICE_CAP_1           (TX_PAGE_CBUS | 0x0081)
#define REG_CBUS_DEVICE_CAP_2           (TX_PAGE_CBUS | 0x0082)
#define REG_CBUS_DEVICE_CAP_3           (TX_PAGE_CBUS | 0x0083)
#define REG_CBUS_DEVICE_CAP_4           (TX_PAGE_CBUS | 0x0084)
#define REG_CBUS_DEVICE_CAP_5           (TX_PAGE_CBUS | 0x0085)
#define REG_CBUS_DEVICE_CAP_6           (TX_PAGE_CBUS | 0x0086)
#define REG_CBUS_DEVICE_CAP_7           (TX_PAGE_CBUS | 0x0087)
#define REG_CBUS_DEVICE_CAP_8           (TX_PAGE_CBUS | 0x0088)
#define REG_CBUS_DEVICE_CAP_9           (TX_PAGE_CBUS | 0x0089)
#define REG_CBUS_DEVICE_CAP_A           (TX_PAGE_CBUS | 0x008A)
#define REG_CBUS_DEVICE_CAP_B           (TX_PAGE_CBUS | 0x008B)
#define REG_CBUS_DEVICE_CAP_C           (TX_PAGE_CBUS | 0x008C)
#define REG_CBUS_DEVICE_CAP_D           (TX_PAGE_CBUS | 0x008D)
#define REG_CBUS_DEVICE_CAP_E           (TX_PAGE_CBUS | 0x008E)
#define REG_CBUS_DEVICE_CAP_F           (TX_PAGE_CBUS | 0x008F)
#define REG_CBUS_SET_INT_0				(TX_PAGE_CBUS | 0x00A0)
#define REG_CBUS_SET_INT_1				(TX_PAGE_CBUS | 0x00A1)
#define REG_CBUS_SET_INT_2				(TX_PAGE_CBUS | 0x00A2)
#define REG_CBUS_SET_INT_3				(TX_PAGE_CBUS | 0x00A3)
#define REG_CBUS_WRITE_STAT_0        	(TX_PAGE_CBUS | 0x00B0)
#define REG_CBUS_WRITE_STAT_1        	(TX_PAGE_CBUS | 0x00B1)
#define REG_CBUS_WRITE_STAT_2        	(TX_PAGE_CBUS | 0x00B2)
#define REG_CBUS_WRITE_STAT_3        	(TX_PAGE_CBUS | 0x00B3)

// DEVCAP we will initialize to
#define	MHL_LOGICAL_DEVICE_MAP		(MHL_DEV_LD_AUDIO | MHL_DEV_LD_VIDEO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_GUI )


