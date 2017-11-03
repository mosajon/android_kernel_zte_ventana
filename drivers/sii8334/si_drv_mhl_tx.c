/*********************************************************************************/
/*  Copyright (c) 2002-2011, Silicon Image, Inc.  All rights reserved.           */
/*  No part of this work may be reproduced, modified, distributed, transmitted,  */
/*  transcribed, or translated into any language or computer format, in any form */
/*  or by any means without written permission of: Silicon Image, Inc.,          */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                         */
/*********************************************************************************/

#include "si_cra.h"
#include "si_mhl_defs.h"
#include "si_mhl_tx_api.h"
#include "si_8334_regs.h"
#include "si_drv_mhl_tx.h"
#include "MHL_SiI8334.h"

extern bool_t	vbusPowerState;
//
// Software power states are a little bit different than the hardware states but
// a close resemblance exists.
//
// D3 matches well with hardware state. In this state we receive RGND interrupts
// to initiate wake up pulse and device discovery
//
// Chip wakes up in D2 mode and interrupts MCU for RGND. Firmware changes the TX
// into D0 mode and sets its own operation mode as POWER_STATE_D0_NO_MHL because
// MHL connection has not yet completed.
//
// For all practical reasons, firmware knows only two states of hardware - D0 and D3.
//
// We move from POWER_STATE_D0_NO_MHL to POWER_STATE_D0_MHL only when MHL connection
// is established.
/*
//
//                             S T A T E     T R A N S I T I O N S
//
//
//                    POWER_STATE_D3                      POWER_STATE_D0_NO_MHL
//                   /--------------\                        /------------\
//                  /                \                      /     D0       \
//                 /                  \                \   /                \
//                /   DDDDDD  333333   \     RGND       \ /   NN  N  OOO     \
//                |   D     D     33   |-----------------|    N N N O   O     |
//                |   D     D  3333    |      IRQ       /|    N  NN  OOO      |
//                \   D     D      33  /               /  \                  /
//                 \  DDDDDD  333333  /                    \   CONNECTION   /
//                  \                /\                     /\             /
//                   \--------------/  \  TIMEOUT/         /  -------------
//                         /|\          \-------/---------/        ||
//                        / | \            500ms\                  ||
//                          |                     \                ||
//                          |  RSEN_LOW                            || MHL_EST
//                           \ (STATUS)                            ||  (IRQ)
//                            \                                    ||
//                             \      /------------\              //
//                              \    /              \            //
//                               \  /                \          //
//                                \/                  \ /      //
//                                 |    CONNECTED     |/======//    
//                                 |                  |\======/
//                                 \   (OPERATIONAL)  / \
//                                  \                /
//                                   \              /
//                                    \-----------/
//                                   POWER_STATE_D0_MHL
//
//
//
*/
#define	POWER_STATE_D3				3
#define	POWER_STATE_D0_NO_MHL		2
#define	POWER_STATE_D0_MHL			0
#define	POWER_STATE_FIRST_INIT		0xFF

#define	TIMER_FOR_MONITORING				(TIMER_0)

#define TX_HW_RESET_PERIOD		10	// 10 ms.
#define TX_HW_RESET_DELAY			100

#define I2C_INACCESSIBLE -1
#define I2C_ACCESSIBLE 1

//
// To remember the current power state.
//
static	uint8_t	fwPowerState = POWER_STATE_FIRST_INIT;

//
// To serialize the RCP commands posted to the CBUS engine, this flag
// is maintained by the function SiiMhlTxDrvSendCbusCommand()
//
static	bool_t		mscCmdInProgress;	// false when it is okay to send a new command
//
// Preserve Downstream HPD status
//
static	uint8_t	dsHpdStatus = 0;


#define WriteByteCBUS(offset,value)  SiiRegWrite(TX_PAGE_CBUS | (uint16_t)offset,value)
#define ReadByteCBUS(offset)         SiiRegRead( TX_PAGE_CBUS | (uint16_t)offset)
#define	SET_BIT(offset,bitnumber)		SiiRegModify(offset,(1<<bitnumber),(1<<bitnumber))
#define	CLR_BIT(offset,bitnumber)		SiiRegModify(offset,(1<<bitnumber),0x00)
//
//
#define	DISABLE_DISCOVERY				CLR_BIT(REG_DISC_CTRL1, 0);
#define	ENABLE_DISCOVERY				SET_BIT(REG_DISC_CTRL1, 0);
//
//	Look for interrupts on INTR4 (Register 0x74)
//		7 = RSVD		(reserved)
//		6 = RGND Rdy	(interested)
//		5 = MHL DISCONNECT	(interested)	
//		4 = CBUS LKOUT	(interested)
//		3 = USB EST		(interested)
//		2 = MHL EST		(interested)
//		1 = RPWR5V Change	(ignore)
//		0 = SCDT Change	(only if necessary)
//
#define	INTR_4_DESIRED_MASK				(BIT0 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7) 
#define	UNMASK_INTR_4_INTERRUPTS		SiiRegWrite(REG_INTR4_MASK, INTR_4_DESIRED_MASK)
#define	MASK_INTR_4_INTERRUPTS			SiiRegWrite(REG_INTR4_MASK, 0x00)

//	Look for interrupts on INTR_5 (Register ??)
//		4 = FIFO UNDERFLOW	(interested)
//		3 = FIFO OVERFLOW	(interested)

#define	INTR_5_DESIRED_MASK				(BIT3 | BIT4) 
#define	UNMASK_INTR_5_INTERRUPTS		SiiRegWrite(REG_INTR5_MASK, INTR_5_DESIRED_MASK)
#define	MASK_INTR_5_INTERRUPTS			SiiRegWrite(REG_INTR5_MASK, 0x00)

//added by garyyuan 20110819
//	Look for interrupts on INTR_1(Register 0x72:0x71)
//		6 = HPD_INT	(interested)

#define	INTR_1_DESIRED_MASK				(BIT5 | BIT6) 
#define	UNMASK_INTR_1_INTERRUPTS		SiiRegWrite(REG_INTR1_MASK, INTR_1_DESIRED_MASK)
#define	MASK_INTR_1_INTERRUPTS			SiiRegWrite(REG_INTR1_MASK, 0x00)


//	Look for interrupts on CBUS:CBUS_INTR_STATUS [0xC8:0x08]
//		7 = RSVD			(reserved)
//		6 = MSC_RESP_ABORT	(interested)
//		5 = MSC_REQ_ABORT	(interested)	
//		4 = MSC_REQ_DONE	(interested)
//		3 = MSC_MSG_RCVD	(interested)
//		2 = DDC_ABORT		(interested)
//		1 = RSVD			(reserved)
//		0 = rsvd			(reserved)
#define	INTR_CBUS1_DESIRED_MASK			(BIT2 | BIT3 | BIT4 | BIT5 | BIT6)
#define	UNMASK_CBUS1_INTERRUPTS			SiiRegWrite(TX_PAGE_CBUS | 0x0009, INTR_CBUS1_DESIRED_MASK)
#define	MASK_CBUS1_INTERRUPTS			SiiRegWrite(TX_PAGE_CBUS | 0x0009, 0x00)

#define	INTR_CBUS2_DESIRED_MASK			(BIT2 | BIT3)
#define	UNMASK_CBUS2_INTERRUPTS			SiiRegWrite(TX_PAGE_CBUS | 0x001F, INTR_CBUS2_DESIRED_MASK)
#define	MASK_CBUS2_INTERRUPTS			SiiRegWrite(TX_PAGE_CBUS | 0x001F, 0x00)

//
// Local scope functions.
//
static void Int1Isr (void);
static int  Int4Isr (void);
static void Int5Isr (void);
static void MhlCbusIsr (void);

static void CbusReset (void);
static void SwitchToD0 (void);
static void SwitchToD3 (void);
static void WriteInitialRegisterValues (void);
static void InitCBusRegs (void);
static void ForceUsbIdSwitchOpen (void);
static void ReleaseUsbIdSwitchOpen (void);
static void MhlTxDrvProcessConnection (void);
static void MhlTxDrvProcessDisconnection (void);


bool_t mhlConnected;
uint8_t g_chipRevId;

static void ProcessScdtStatusChange (void);

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvAcquireUpstreamHPDControl
//
// Acquire the direct control of Upstream HPD.
//
void SiiMhlTxDrvAcquireUpstreamHPDControl (void)
{
	// set reg_hpd_out_ovr_en to first control the hpd
	SET_BIT(REG_INT_CTRL, 4);
	TX_DEBUG_PRINT(("Drv: Upstream HPD Acquired.\n"));
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow
//
// Acquire the direct control of Upstream HPD.
//
void SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow (void)
{
	// set reg_hpd_out_ovr_en to first control the hpd and clear reg_hpd_out_ovr_val
 	SiiRegModify(REG_INT_CTRL, BIT5 | BIT4, BIT4);	// Force upstream HPD to 0 when not in MHL mode.
	TX_DEBUG_PRINT(("Drv: Upstream HPD Acquired - driven low.\n"));
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvReleaseUpstreamHPDControl
//
// Release the direct control of Upstream HPD.
//
void SiiMhlTxDrvReleaseUpstreamHPDControl (void)
{
   	// Un-force HPD (it was kept low, now propagate to source
	// let HPD float by clearing reg_hpd_out_ovr_en
   	CLR_BIT(REG_INT_CTRL, 4);
	TX_DEBUG_PRINT(("Drv: Upstream HPD released.\n"));
}

////////////////////////////////////////////////////////////////////
//
// E X T E R N A L L Y    E X P O S E D   A P I    F U N C T I O N S
//
////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxChipInitialize
//
// Chip specific initialization.
// This function is for SiI 8332/8336 Initialization: HW Reset, Interrupt enable.
//
//
//////////////////////////////////////////////////////////////////////////////

bool_t SiiMhlTxChipInitialize (void)
{
	mhlConnected = false;
	mscCmdInProgress = false;	// false when it is okay to send a new command
	dsHpdStatus = 0;
	fwPowerState = POWER_STATE_D0_MHL;
    //SI_OS_DISABLE_DEBUG_CHANNEL(SII_OSAL_DEBUG_SCHEDULER);

    g_chipRevId = SiiRegRead(TX_PAGE_L0 | 0x04);

	// then wait 100ms per MHL spec
	HalTimerWait(TX_HW_RESET_DELAY);
	TX_DEBUG_PRINT(("Drv: SiiMhlTxChipInitialize: %02X%02X%02x\n",
						g_chipRevId,
					SiiRegRead(TX_PAGE_L0 | 0x03),
					SiiRegRead(TX_PAGE_L0 | 0x02)));

	// setup device registers. Ensure RGND interrupt would happen.
	WriteInitialRegisterValues();

	// Setup interrupt masks for all those we are interested.
	//UNMASK_INTR_4_INTERRUPTS;
	//UNMASK_INTR_1_INTERRUPTS;

	// CBUS interrupts are unmasked after performing the reset.
	// UNMASK_CBUS1_INTERRUPTS;
	// UNMASK_CBUS2_INTERRUPTS;

	//
	// Allow regular operation - i.e. pinAllowD3 is high so we do enter
	// D3 first time. Later on, SiIMon shall control this GPIO.
	//
	//pinAllowD3 = 1;

	SwitchToD3();

	return true;
}
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
//		CBUS 		= responder to peer messages
//					  Especially for DCAP etc time based events
//
void SiiMhlTxDeviceIsr (void)
{
	uint8_t intMStatus, i; //master int status
	//
	// Look at discovery interrupts if not yet connected.
	//

	i=0;

	do
	{
		if( POWER_STATE_D0_MHL != fwPowerState )
		{
			//
			// Check important RGND, MHL_EST, CBUS_LOCKOUT and SCDT interrupts
			// During D3 we only get RGND but same ISR can work for both states
			//
			if (I2C_INACCESSIBLE == Int4Isr())
			{
				return; // don't do any more I2C traffic until the next interrupt.
			}
		}
		else if( POWER_STATE_D0_MHL == fwPowerState )
		{

			if (I2C_INACCESSIBLE == Int4Isr())
			{
				return; // don't do any more I2C traffic until the next interrupt.
			}

			if(POWER_STATE_D0_MHL == fwPowerState)
			{
				
#if 0
				TX_DEBUG_PRINT(("********* EXITING ISR *************\n"));
				TX_DEBUG_PRINT(("Drv: INT1 Status = %02X\n", (int) SiiRegRead((TX_PAGE_L0 | 0x0071))));
				TX_DEBUG_PRINT(("Drv: INT2 Status = %02X\n", (int) SiiRegRead((TX_PAGE_L0 | 0x0072))));
				TX_DEBUG_PRINT(("Drv: INT3 Status = %02X\n", (int) SiiRegRead((TX_PAGE_L0 | 0x0073))));
				TX_DEBUG_PRINT(("Drv: INT4 Status = %02X\n", (int) SiiRegRead((TX_PAGE_L0 | 0x0073))));
				TX_DEBUG_PRINT(("Drv: INT5 Status = %02X\n", (int) SiiRegRead(REG_INTR4)));
				TX_DEBUG_PRINT(("Drv: INT5 Status = %02X\n", (int) SiiRegRead(REG_INTR5)));
			
			
				TX_DEBUG_PRINT(("Drv: cbusInt Status = %02X\n", (int) SiiRegRead(TX_PAGE_CBUS | 0x0008)));
			
				TX_DEBUG_PRINT(("Drv: CBUS INTR_2: 0x1E: %02X\n", (int) SiiRegRead(TX_PAGE_CBUS | 0x001E)));
				TX_DEBUG_PRINT(("Drv: A0 INT Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00A0))));
				TX_DEBUG_PRINT(("Drv: A1 INT Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00A1))));
				TX_DEBUG_PRINT(("Drv: A2 INT Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00A2))));
				TX_DEBUG_PRINT(("Drv: A3 INT Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00A3))));
			
				TX_DEBUG_PRINT(("Drv: B0 STATUS Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00B0))));
				TX_DEBUG_PRINT(("Drv: B1 STATUS Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00B1))));
				TX_DEBUG_PRINT(("Drv: B2 STATUS Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00B2))));
				TX_DEBUG_PRINT(("Drv: B3 STATUS Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00B3))));
			
				TX_DEBUG_PRINT(("Drv: E0 STATUS Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00E0))));
				TX_DEBUG_PRINT(("Drv: E1 STATUS Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00E1))));
				TX_DEBUG_PRINT(("Drv: E2 STATUS Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00E2))));
				TX_DEBUG_PRINT(("Drv: E3 STATUS Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00E3))));
			
				TX_DEBUG_PRINT(("Drv: F0 INT Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00F0))));
				TX_DEBUG_PRINT(("Drv: F1 INT Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00F1))));
				TX_DEBUG_PRINT(("Drv: F2 INT Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00F2))));
				TX_DEBUG_PRINT(("Drv: F3 INT Set = %02X\n", (int) SiiRegRead((TX_PAGE_CBUS | 0x00F3))));
				TX_DEBUG_PRINT(("********* END OF EXITING ISR *************\n"));
	#endif
				// If the Int4Isr handler didn't move the transmitter to D3 as the
				// result of a cable disconnection continue to check other interrupt
				// sources.
				Int5Isr();
	
				// Check for any peer messages for DCAP_CHG etc
				// Dispatch to have the CBUS module working only once connected.
				MhlCbusIsr();
				Int1Isr();
				
			}
		}

		intMStatus = SiiRegRead(TX_PAGE_L0 | 0x0070);	// read status
		if(0xFF == intMStatus)
		{
			intMStatus = 0;
			TX_DEBUG_PRINT(("\nDrv: EXITING ISR DUE TO intMStatus - 0xFF loop = [%02X] intMStatus = [%02X] \n\n", (int) i, (int)intMStatus));
		}
		i++;

		intMStatus &= 0x01; //RG mask bit 0
	} while (intMStatus);
}


///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvTmdsControl
//
// Control the TMDS output. MhlTx uses this to support RAP content on and off.
//
void SiiMhlTxDrvTmdsControl (bool_t enable)
{
	if( enable )
	{
		SET_BIT(TX_PAGE_L0 | 0x0080, 4);
	    TX_DEBUG_PRINT(("Drv:%d TMDS Output Enabled\n",(int)__LINE__));
        SiiMhlTxDrvReleaseUpstreamHPDControl();  // this triggers an EDID read
	}
	else
	{
		CLR_BIT(TX_PAGE_L0 | 0x0080, 4);
	    TX_DEBUG_PRINT(("Drv:%d TMDS Ouput Disabled\n",(int)__LINE__));
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvNotifyEdidChange
//
// MhlTx may need to inform upstream device of an EDID change. This can be
// achieved by toggling the HDMI HPD signal or by simply calling EDID read
// function.
//
void SiiMhlTxDrvNotifyEdidChange (void)
{
    TX_DEBUG_PRINT(("Drv: SiiMhlTxDrvNotifyEdidChange\n"));
	//
	// Prepare to toggle HPD to upstream
	//
    SiiMhlTxDrvAcquireUpstreamHPDControl();

	// reg_hpd_out_ovr_val = LOW to force the HPD low
	CLR_BIT(REG_INT_CTRL, 5);

	// wait a bit
	HalTimerWait(110);

	// release HPD back to high by reg_hpd_out_ovr_val = HIGH
	SET_BIT(REG_INT_CTRL, 5);

    // release control to allow transcoder to modulate for CLR_HPD and SET_HPD
    SiiMhlTxDrvReleaseUpstreamHPDControl();
}
//------------------------------------------------------------------------------
// Function:    SiiMhlTxDrvSendCbusCommand
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
//------------------------------------------------------------------------------

bool_t SiiMhlTxDrvSendCbusCommand (cbus_req_t *pReq)
{
    bool_t  success = true;

    uint8_t i, startbit;

	//
	// If not connected, return with error
	//
	if( (POWER_STATE_D0_MHL != fwPowerState ) || (mscCmdInProgress))
	{
	    TX_DEBUG_PRINT(("Error: Drv:%d fwPowerState: %02X, or CBUS(0x0A):%02X mscCmdInProgress = %d\n",(int)__LINE__,
			(int) fwPowerState,
			(int) SiiRegRead(TX_PAGE_CBUS | 0x000A),
			(int) mscCmdInProgress));

   		return false;
	}
	// Now we are getting busy
	mscCmdInProgress	= true;

    TX_DEBUG_PRINT(("Drv:%d Sending MSC command %02X, %02X, %02X, %02X\n",(int)__LINE__,
			(int)pReq->command, 
			(int)(pReq->offsetData),
		 	(int)pReq->payload_u.msgData[0],
		 	(int)pReq->payload_u.msgData[1]));

    /****************************************************************************************/
    /* Setup for the command - write appropriate registers and determine the correct        */
    /*                         start bit.                                                   */
    /****************************************************************************************/

	// Set the offset and outgoing data byte right away
	SiiRegWrite(TX_PAGE_CBUS | 0x0013, pReq->offsetData); 	// set offset
	SiiRegWrite(TX_PAGE_CBUS | 0x0014, pReq->payload_u.msgData[0]);
	
    startbit = 0x00;
    switch ( pReq->command )
    {
		case MHL_SET_INT:	// Set one interrupt register = 0x60
			startbit = MSC_START_BIT_WRITE_REG;
			break;

        case MHL_WRITE_STAT:	// Write one status register = 0x60 | 0x80
            startbit = MSC_START_BIT_WRITE_REG;
            break;

        case MHL_READ_DEVCAP:	// Read one device capability register = 0x61
            startbit = MSC_START_BIT_READ_REG;
            break;

 		case MHL_GET_STATE:			// 0x62 -
		case MHL_GET_VENDOR_ID:		// 0x63 - for vendor id	
		case MHL_SET_HPD:			// 0x64	- Set Hot Plug Detect in follower
		case MHL_CLR_HPD:			// 0x65	- Clear Hot Plug Detect in follower
		case MHL_GET_SC1_ERRORCODE:		// 0x69	- Get channel 1 command error code
		case MHL_GET_DDC_ERRORCODE:		// 0x6A	- Get DDC channel command error code.
		case MHL_GET_MSC_ERRORCODE:		// 0x6B	- Get MSC command error code.
		case MHL_GET_SC3_ERRORCODE:		// 0x6D	- Get channel 3 command error code.
			SiiRegWrite(REG_CBUS_PRI_ADDR_CMD, pReq->command );
            startbit = MSC_START_BIT_MSC_CMD;
            break;

        case MHL_MSC_MSG:
			SiiRegWrite(REG_CBUS_PRI_WR_DATA_2ND, pReq->payload_u.msgData[1]);
			SiiRegWrite(REG_CBUS_PRI_ADDR_CMD, pReq->command );
            startbit = MSC_START_BIT_VS_CMD;
            break;

        case MHL_WRITE_BURST:
            SiiRegWrite(REG_MSC_WRITE_BURST_LEN, pReq->length -1 );

            // Now copy all bytes from array to local scratchpad
            if (NULL == pReq->payload_u.pdatabytes)
            {
                TX_DEBUG_PRINT(("\nDrv:%d Put pointer to WRITE_BURST data in req.pdatabytes!!!\n\n",(int)__LINE__));
				success = false;
            }
            else
            {
	            uint8_t *pData = pReq->payload_u.pdatabytes;
                TX_DEBUG_PRINT(("\nDrv:%d Writing data into scratchpad\n\n",(int)__LINE__));
            for ( i = 0; i < pReq->length; i++ )
            {
					SiiRegWrite(REG_CBUS_SCRATCHPAD_0 + i, *pData++ );
	            }
			}
            startbit = MSC_START_BIT_WRITE_BURST;
            break;

        default:
            success = false;
            break;
    }

    /****************************************************************************************/
    /* Trigger the CBUS command transfer using the determined start bit.                    */
    /****************************************************************************************/

    if ( success )
    {
        SiiRegWrite(REG_CBUS_PRI_START, startbit );
    }
    else
    {
        TX_DEBUG_PRINT(("\nDrv:%d SiiMhlTxDrvSendCbusCommand failed\n\n",(int)__LINE__));
    }

    return( success );
}

bool_t SiiMhlTxDrvCBusBusy (void)
{
    return mscCmdInProgress ? true :false;
}

///////////////////////////////////////////////////////////////////////////
// WriteInitialRegisterValues
//
//
///////////////////////////////////////////////////////////////////////////
static void WriteInitialRegisterValues (void)
{
	//TX_DEBUG_PRINT(("Drv: WriteInitialRegisterValues\n"));

	// Power Up
	SiiRegWrite(TX_PAGE_L1 | 0x003D, 0x3F);			// Power up CVCC 1.2V core
	SiiRegWrite(TX_PAGE_2 | 0x0011, 0x01);			// Enable TxPLL Clock
	SiiRegWrite(TX_PAGE_2 | 0x0012, 0x11);			// Enable Tx Clock Path & Equalizer

	SiiRegWrite(REG_MHLTX_CTL1, 0x10); // TX Source termination ON
	SiiRegWrite(REG_MHLTX_CTL6, 0xBC); // Enable 1X MHL clock output
	SiiRegWrite(REG_MHLTX_CTL2, 0x3C); // TX Differential Driver Config
	SiiRegWrite(REG_MHLTX_CTL4, 0xD9); 
	SiiRegWrite(REG_MHLTX_CTL8, 0x02); // PLL BW Control

	// Analog PLL Control
	SiiRegWrite(TX_PAGE_L0 | 0x0080, 0x00);			// Enable Rx PLL clock
	SiiRegWrite(TX_PAGE_L0 | 0x00F8, 0x0C);
    SiiRegWrite(TX_PAGE_L0 | 0x0085, 0x02);

	SiiRegWrite(TX_PAGE_2 | 0x0000, 0x00);
	SiiRegWrite(TX_PAGE_2 | 0x0013, 0x60);

	SiiRegWrite(TX_PAGE_2 | 0x0017, 0x03);			// PLL Calrefsel
	SiiRegWrite(TX_PAGE_2 | 0x001A, 0x20);			// VCO Cal
	SiiRegWrite(TX_PAGE_2 | 0x0022, 0xE0);			// Auto EQ
	SiiRegWrite(TX_PAGE_2 | 0x0023, 0xC0);			// Auto EQ
	SiiRegWrite(TX_PAGE_2 | 0x0024, 0xA0);			// Auto EQ
	SiiRegWrite(TX_PAGE_2 | 0x0025, 0x80);			// Auto EQ
	SiiRegWrite(TX_PAGE_2 | 0x0026, 0x60);			// Auto EQ
	SiiRegWrite(TX_PAGE_2 | 0x0027, 0x40);			// Auto EQ
	SiiRegWrite(TX_PAGE_2 | 0x0028, 0x20);			// Auto EQ
	SiiRegWrite(TX_PAGE_2 | 0x0029, 0x00);			// Auto EQ

	SiiRegWrite(TX_PAGE_2 | 0x0031, 0x0A);			// Rx PLL BW ~ 4MHz
	SiiRegWrite(TX_PAGE_2 | 0x0045, 0x06);			// Rx PLL BW value from I2C

	SiiRegWrite(TX_PAGE_2 | 0x004B, 0x06);
	SiiRegWrite(TX_PAGE_2 | 0x004C, 0x60);			// Manual zone control
	SiiRegWrite(TX_PAGE_2 | 0x004C, 0xE0);			// Manual zone control
	SiiRegWrite(TX_PAGE_2 | 0x004D, 0x00);			// PLL Mode Value

	SiiRegWrite(TX_PAGE_L0 | 0x0008, 0x35);			// bring out from power down (script moved this here from above)

	SiiRegWrite(REG_DISC_CTRL2, 0xAD);
	SiiRegWrite(REG_DISC_CTRL5, 0x55);				// 1.8V CBUS VTH
	SiiRegWrite(REG_DISC_CTRL6, 0x11);				// RGND & single discovery attempt (RGND blocking)
	SiiRegWrite(REG_DISC_CTRL8, 0x82);				// Ignore VBUS
	SiiRegWrite(REG_DISC_CTRL9, 0x24);				// No OTG, Discovery pulse proceed, Wake pulse not bypassed
	SiiRegWrite(REG_DISC_CTRL4, 0x0C);				// Pull-up resistance off for IDLE state.
	SiiRegWrite(REG_DISC_CTRL1, 0x27);				// Enable CBUS discovery
	SiiRegWrite(REG_DISC_CTRL7, 0x20);				// use 1K and 2K setting
	SiiRegWrite(REG_DISC_CTRL3, 0x86);				// MHL CBUS discovery

	CLR_BIT(REG_INT_CTRL, 6);//change hpd out pin from defult open-drain to push-pull by garyyuan
	if (fwPowerState != TX_POWER_STATE_D3) {			// Don't force HPD to 0 during wake-up from D3
		SiiRegModify(REG_INT_CTRL, BIT5 | BIT4, BIT4);	// Force HPD to 0 when not in MHL mode.
		}

	SiiRegWrite(REG_SRST, 0x04); 					// Enable Auto soft reset on SCDT = 0

	SiiRegWrite(TX_PAGE_L0 | 0x000D, 0x1C); 		// HDMI Transcode mode enable

	CbusReset();

	InitCBusRegs();
}

///////////////////////////////////////////////////////////////////////////
// InitCBusRegs
//
///////////////////////////////////////////////////////////////////////////
static void InitCBusRegs (void)
{
	uint8_t		regval;

	//TX_DEBUG_PRINT(("Drv: InitCBusRegs\n"));

	SiiRegWrite(TX_PAGE_CBUS | 0x0007, 0x32); 			// Increase DDC translation layer timer
	SiiRegWrite(TX_PAGE_CBUS | 0x0036, 0x0C); 			// Drive High Time.
	SiiRegWrite(TX_PAGE_CBUS | 0x0039, 0x30); 			// Use programmed timing.
	SiiRegWrite(TX_PAGE_CBUS | 0x0040, 0x03); 			// CBUS Drive Strength

	// Setup our devcap
	SiiRegWrite(TX_PAGE_CBUS | 0x0080, MHL_DEV_ACTIVE);
	SiiRegWrite(TX_PAGE_CBUS | 0x0081, MHL_VERSION);
	SiiRegWrite(TX_PAGE_CBUS | 0x0082, (MHL_DEV_CAT_SOURCE));
	SiiRegWrite(TX_PAGE_CBUS | 0x0083, 0);
	SiiRegWrite(TX_PAGE_CBUS | 0x0084, 0);						
	SiiRegWrite(TX_PAGE_CBUS | 0x0085, MHL_DEV_VID_LINK_SUPPRGB444);
	SiiRegWrite(TX_PAGE_CBUS | 0x0086, MHL_DEV_AUD_LINK_2CH);
	SiiRegWrite(TX_PAGE_CBUS | 0x0087, 0);										// not for source
	SiiRegWrite(TX_PAGE_CBUS | 0x0088, MHL_LOGICAL_DEVICE_MAP);
	SiiRegWrite(TX_PAGE_CBUS | 0x0089, 0);										// not for source
	SiiRegWrite(TX_PAGE_CBUS | 0x008A, (MHL_FEATURE_RCP_SUPPORT | MHL_FEATURE_RAP_SUPPORT));
	SiiRegWrite(TX_PAGE_CBUS | 0x008B, 0);
	SiiRegWrite(TX_PAGE_CBUS | 0x008C, 0);										// reserved
	SiiRegWrite(TX_PAGE_CBUS | 0x008D, MHL_SCRATCHPAD_SIZE);
	SiiRegWrite(TX_PAGE_CBUS | 0x008E, (uint8_t)MHL_INT_AND_STATUS_SIZE);
	SiiRegWrite(TX_PAGE_CBUS | 0x008F, 0);										//reserved

	// Make bits 2,3 (initiator timeout) to 1,1 for register CBUS_LINK_CONTROL_2
	regval = SiiRegRead(REG_CBUS_LINK_CONTROL_2);
	regval = (regval | 0x0C);
	SiiRegWrite(REG_CBUS_LINK_CONTROL_2, regval);

	 // Clear legacy bit on Wolverine TX.
    regval = SiiRegRead(REG_MSC_TIMEOUT_LIMIT);
    SiiRegWrite(REG_MSC_TIMEOUT_LIMIT, (regval & MSC_TIMEOUT_LIMIT_MSB_MASK));

	// Set NMax to 1
	SiiRegWrite(REG_CBUS_LINK_CONTROL_1, 0x01);
    SiiRegModify(TX_PAGE_CBUS | 0x0024, BIT4,BIT4);  // disallow vendor specific commands

}

///////////////////////////////////////////////////////////////////////////
//
// ForceUsbIdSwitchOpen
//
///////////////////////////////////////////////////////////////////////////
static void ForceUsbIdSwitchOpen (void)
{
	SiiRegWrite(REG_DISC_CTRL1, 0x26);								// Disable CBUS discovery
	SiiRegModify(REG_DISC_CTRL6, BIT6, BIT6);				// Force USB ID switch to open
	SiiRegWrite(REG_DISC_CTRL3, 0x86);
	SiiRegModify(REG_INT_CTRL, BIT5 | BIT4, BIT4);		// Force HPD to 0 when not in Mobile HD mode.
}
///////////////////////////////////////////////////////////////////////////
//
// ReleaseUsbIdSwitchOpen
//
///////////////////////////////////////////////////////////////////////////
static void ReleaseUsbIdSwitchOpen (void)
{
	HalTimerWait(50); // per spec
	SiiRegModify(REG_DISC_CTRL6, BIT6, 0x00);
	SiiRegModify(REG_DISC_CTRL1, BIT0, BIT0);				// Enable discovery
}


///////////////////////////////////////////////////////////////////////////
// ProcessRgnd
//
// H/W has detected impedance change and interrupted.
// We look for appropriate impedance range to call it MHL and enable the
// hardware MHL discovery logic. If not, disable MHL discovery to allow
// USB to work appropriately.
//
// In current chip a firmware driven slow wake up pulses are sent to the
// sink to wake that and setup ourselves for full D0 operation.
///////////////////////////////////////////////////////////////////////////
void ProcessRgnd (void)
{
	uint8_t rgndImpedance;
	//
	// Impedance detection has completed - process interrupt
	//
	rgndImpedance = SiiRegRead(REG_DISC_STAT2) & 0x03;
	TX_DEBUG_PRINT(("Drv: RGND = %02X : \n", (int)rgndImpedance));

	//
	// 00, 01 or 11 means USB.
	// 10 means 1K impedance (MHL)
	//
	// If 1K, then only proceed with wake up pulses
	if (0x02 == rgndImpedance)
	{
		SiiRegModify(REG_DISC_CTRL4, BIT7 | BIT6 , BIT7);				// discovery Pull-up 10K for MHL state.
		SiiRegModify(REG_DISC_CTRL9, BIT0, BIT0);
		TX_DEBUG_PRINT(("(MHL Device)\n"));

		//The sequence of events during MHL discovery is as follows:
		//	(i) SiI9244 blocks on RGND interrupt (Page0:0x74[6]).
		//	(ii) System firmware turns off its own VBUS if present.
		//	(iii) System firmware waits for about 200ms (spec: TVBUS_CBUS_STABLE, 100 - 1000ms), then checks for the presence of
		//		VBUS from the Sink.
		//	(iv) If VBUS is present then system firmware proceed to drive wake pulses to the Sink as described in previous
		//		section.
		//	(v) If VBUS is absent the system firmware turns on its own VBUS, wait for an additional 200ms (spec:
		//		TVBUS_OUT_TO_STABLE, 100 - 1000ms), and then proceed to drive wake pulses to the Sink as described in above.

		// AP need to check VBUS power present or absent in here 	// by oscar 20110527
		
#if (VBUS_POWER_CHK == ENABLE)			// Turn on VBUS output.
		AppVbusControl( vbusPowerState = false );
#endif

		TX_DEBUG_PRINT(("[MHL]: Waiting T_SRC_VBUS_CBUS_TO_STABLE (%d ms)\n", (int)T_SRC_VBUS_CBUS_TO_STABLE));
		//HalTimerWait(T_SRC_VBUS_CBUS_TO_STABLE);
	}
	else
	{
		SiiRegModify(REG_DISC_CTRL9, BIT3, BIT3);	// USB Established
		TX_DEBUG_PRINT(("(Non-MHL Device)\n"));
	}
}


////////////////////////////////////////////////////////////////////
// SwitchToD0
// This function performs s/w as well as h/w state transitions.
//
// Chip comes up in D2. Firmware must first bring it to full operation
// mode in D0.
////////////////////////////////////////////////////////////////////
void SwitchToD0 (void)
{
	TX_DEBUG_PRINT(("Drv: Switch to D0\n"));
//	TX_DEBUG_PRINT(("[%d] Drv: Switch To Full power mode (D0)\n",
//							(int) (HalTimerElapsed( ELAPSED_TIMER ) * MONITORING_PERIOD)) );

	//
	// WriteInitialRegisterValues switches the chip to full power mode.
	//
	WriteInitialRegisterValues();

	// Force Power State to ON
	SiiRegWrite(REG_DISC_CTRL1, 0x25);					// Force Power State to ON
	SiiRegModify(TPI_DEVICE_POWER_STATE_CTRL_REG, TX_POWER_STATE_MASK, 0x00);

	fwPowerState = POWER_STATE_D0_NO_MHL;
}
////////////////////////////////////////////////////////////////////
// SwitchToD3
//
// This function performs s/w as well as h/w state transitions.
//
////////////////////////////////////////////////////////////////////
void SwitchToD3 (void)
{
	if(POWER_STATE_D3 != fwPowerState)
	{
		TX_DEBUG_PRINT(("Drv: Switch To D3\n"));
		//SiiRegModify(REG_DISC_CTRL6,  BIT4, BIT4); //Block RGND INT in Discovery SM. added by garyyuan 20100804
//		TX_DEBUG_PRINT(("[%d] Drv: Switch To D3: pinAllowD3 = %d\n",
//							(int) (HalTimerElapsed( ELAPSED_TIMER ) * MONITORING_PERIOD), (int) pinAllowD3 ) );

		//pinM2uVbusCtrlM = 1;
		//pinMhlConn = 1;
		//pinUsbConn = 0;

//		ForceUsbIdSwitchOpen();

		//
		// To allow RGND engine to operate correctly.
		// So when moving the chip from D0 MHL connected to D3 the values should be
		// 94[1:0] = 00  reg_cbusmhl_pup_sel[1:0] should be set for open
		// 93[7:6] = 00  reg_cbusdisc_pup_sel[1:0] should be set for open
		// 93[5:4] = 00  reg_cbusidle_pup_sel[1:0] = open (default)
		//
		// Disable CBUS pull-up during RGND measurement
//		I2C_WriteByte(PAGE_0_0X72, 0x93, 0x04);
//		ReadModifyWritePage0(0x93, BIT_7 | BIT_6 | BIT_5 | BIT_4, 0);

//		ReadModifyWritePage0(0x94, BIT_1 | BIT_0, 0);

		// 1.8V CBUS VTH & GND threshold
//		I2C_WriteByte(PAGE_0_0X72, 0x94, 0x64);

//		ReleaseUsbIdSwitchOpen();

        SiiRegModify(REG_DISC_CTRL4, BIT7 | BIT6,0);

		// Force HPD to 0 when not in MHL mode.
        SiiMhlTxDrvAcquireUpstreamHPDControlDriveLow();

		// Change TMDS termination to high impedance on disconnection; reduce leakage to QCOM HDMI out, due to QCOM HDMI out can't be truly tri-stated. Each TMDS pin has about 0.1125mA leakage from AVCC33.
		// Bits 1:0 set to 11
		SiiRegWrite(TX_PAGE_2 | 0x0001, 0x03);


		SiiRegWrite(REG_MHLTX_CTL1, 0xD0);

		//
		// GPIO controlled from SiIMon can be utilized to disallow
		// low power mode, thereby allowing SiIMon to debug register contents.
		// Otherwise SiIMon reads all registers as 0xFF
		//
//		if(pinAllowD3)
//		{
			//
			// Change state to D3 by clearing bit 0 of 3D (SW_TPI, Page 1) register
			// ReadModifyWriteIndexedRegister(INDEXED_PAGE_1, 0x3D, BIT_0, 0x00);
			//
			CLR_BIT(TX_PAGE_L1 | 0x003D, 0);

			fwPowerState = POWER_STATE_D3;
//		}

#if (VBUS_POWER_CHK == ENABLE)		// Turn VBUS power off when switch to D3(cable out)
	if( vbusPowerState == false )
	{
		AppVbusControl( vbusPowerState = true );
	}
#endif
	}
		else
		{
			fwPowerState = POWER_STATE_D0_NO_MHL;
		}
}

////////////////////////////////////////////////////////////////////
// Int1Isr
//
//
//	Look for interrupts on INTR_1(Register 0x72:0x71)
//		6 = HPD_INT	(interested)
////////////////////////////////////////////////////////////////////

static void Int1Isr(void)
{
	uint8_t regIntr1;
    regIntr1 = SiiRegRead(REG_INTR1);
    if (regIntr1)
    {
        // Clear all interrupts coming from this register.
        SiiRegWrite(REG_INTR1,regIntr1);

        if (BIT6 & regIntr1)
        {
        uint8_t cbusStatus;
        	//
        	// Check if a SET_HPD came from the downstream device.
        	//
        	cbusStatus = SiiRegRead(TX_PAGE_CBUS | 0x000D);

        	// CBUS_HPD status bit
        	if(BIT6 & (dsHpdStatus ^ cbusStatus))
        	{
            uint8_t status = cbusStatus & BIT6;
        		TX_DEBUG_PRINT(("Drv: Downstream HPD changed to: %02X\n", (int) cbusStatus));

        		// Inform upper layer of change in Downstream HPD
        		SiiMhlTxNotifyDsHpdChange( status );

                if (status)
                {
                    SiiMhlTxDrvReleaseUpstreamHPDControl();  // this triggers an EDID read if control has not yet been released
                }

        		// Remember
        		dsHpdStatus = cbusStatus;
        	}
        }
    }
}


////////////////////////////////////////////////////////////////////
// Int4Isr
//
//
//	Look for interrupts on INTR4 (Register 0x74)
//		7 = RSVD		(reserved)
//		6 = RGND Rdy	(interested)
//		5 = VBUS Low	(ignore)	
//		4 = CBUS LKOUT	(interested)
//		3 = USB EST		(interested)
//		2 = MHL EST		(interested)
//		1 = RPWR5V Change	(ignore)
//		0 = SCDT Change	(interested during D0)
////////////////////////////////////////////////////////////////////
static int Int4Isr (void)
{
	uint8_t int4Status;

	int4Status = SiiRegRead(REG_INTR4);	// read status
	if( int4Status )
	{
	    TX_DEBUG_PRINT(("Drv: int4Status: 0x%02X\n", (int) int4Status));
	}

	// When I2C is inoperational (D3) and a previous interrupt brought us here, do nothing.
	if(0xFF == int4Status || 0x87 == int4Status|| 0x38 == int4Status)
	{
		return I2C_INACCESSIBLE;
	}

	if((int4Status & BIT0)&&( POWER_STATE_D0_MHL == fwPowerState )) // SCDT Status Change
	{
		if (g_chipRevId < 1)
			ProcessScdtStatusChange();
	}

	// process MHL_EST interrupt
	if(int4Status & BIT2) // MHL_EST_INT
	{
		MhlTxDrvProcessConnection();
	}

	// process USB_EST interrupt
	else if(int4Status & BIT3)
	{
		TX_DEBUG_PRINT(("Drv: uUSB-A type device detected.\n"));
		SiiRegWrite(REG_DISC_STAT2, 0x80);	// Exit D3 via CBUS falling edge
		SwitchToD3();
		return I2C_INACCESSIBLE;
	}

	if (int4Status & BIT5)
	{
		MhlTxDrvProcessDisconnection();


		// Call back into the MHL component to give it a chance to
		// post process the disconnection event.
		MhlTxDriveStates();
		return I2C_INACCESSIBLE;
	}

	if((POWER_STATE_D0_MHL != fwPowerState) && (int4Status & BIT6))
	{
		// Switch to full power mode.
		SwitchToD0();

	//
	// If a sink is connected but not powered on, this interrupt can keep coming
	// Determine when to go back to sleep. Say after 1 second of this state.
	//
	// Check RGND register and send wake up pulse to the peer
	//
	ProcessRgnd();
	}

	// Can't succeed at these in D3

	if(fwPowerState != POWER_STATE_D3)
	{
		// CBUS Lockout interrupt?
		if (int4Status & BIT4)
		{
			TX_DEBUG_PRINT(("Drv: CBus Lockout\n"));

			ForceUsbIdSwitchOpen();
			ReleaseUsbIdSwitchOpen();
		}
    }
	SiiRegWrite(REG_INTR4, int4Status); // clear all interrupts
	return I2C_ACCESSIBLE;
}

////////////////////////////////////////////////////////////////////
// Int5Isr
//
//
//	Look for interrupts on INTR5
//		7 = 
//		6 = 
//		5 = 
//		4 = 
//		3 = 
//		2 = 
//		1 = 
//		0 = 
////////////////////////////////////////////////////////////////////
static void Int5Isr (void)
{
	uint8_t int5Status;

	int5Status = SiiRegRead(REG_INTR5);	// read status
#if 0
	if((int5Status & BIT4) || (int5Status & BIT3)) // FIFO U/O
	{
		TX_DEBUG_PRINT(("** int5Status = %02X; Applying MHL FIFO Reset\n", (int)int5Status));
		SiiRegWrite(REG_SRST, 0x94);
		SiiRegWrite(REG_SRST, 0x84);
	}
#endif
	SiiRegWrite(REG_INTR5, int5Status);	// clear all interrupts
}

///////////////////////////////////////////////////////////////////////////
//
// MhlTxDrvProcessConnection
//
///////////////////////////////////////////////////////////////////////////
static void MhlTxDrvProcessConnection (void)
{
	TX_DEBUG_PRINT (("Drv: MHL Cable Connected. CBUS:0x0A = %02X\n", (int) SiiRegRead(TX_PAGE_CBUS | 0x000A)));

	if( POWER_STATE_D0_MHL == fwPowerState )
	{
		return;
	}
	// VBUS control gpio
	//pinM2uVbusCtrlM = 0;
	//pinMhlConn = 0;
	//pinUsbConn = 1;

	//
	// Discovery over-ride: reg_disc_ovride	
	//
	SiiRegWrite(REG_MHLTX_CTL1, 0x10);

	fwPowerState = POWER_STATE_D0_MHL;

	//
	// Increase DDC translation layer timer (uint8_t mode)
	// Setting DDC Byte Mode
	//
	SiiRegWrite(TX_PAGE_CBUS | 0x0007, 0x32);

	// Enable segment pointer safety
	//SET_BIT(0x0C44, 1);

	// Un-force HPD (it was kept low, now propagate to source
	//CLR_BIT(REG_INT_CTRL, 4);

	// Enable TMDS
	//SiiMhlTxDrvTmdsControl( true );

	
	// Change TMDS termination to 50 ohm termination (default)
	// Bits 1:0 set to 00
	SiiRegWrite(TX_PAGE_2 | 0x0001, 0x00);

	// Keep the discovery enabled. Need RGND interrupt
	// SET_BIT(PAGE_0_0X72, 0x90, 0);
	ENABLE_DISCOVERY;

	// Wait T_SRC_RXSENSE_CHK ms to allow connection/disconnection to be stable (MHL 1.0 specs)
//	TX_DEBUG_PRINT (("[%d] Drv: Wait T_SRC_RXSENSE_CHK (%d ms) before checking RSEN\n",
//							(int) (HalTimerElapsed( ELAPSED_TIMER ) * MONITORING_PERIOD),
//							(int) T_SRC_RXSENSE_CHK) );

	//
	// Ignore RSEN interrupt for T_SRC_RXSENSE_CHK duration.
	// Get the timer started
	//
//	HalTimerSet(TIMER_TO_DO_RSEN_CHK, T_SRC_RXSENSE_CHK);

	// Notify upper layer of cable connection
	SiiMhlTxNotifyConnection(mhlConnected = true);
}
///////////////////////////////////////////////////////////////////////////
//
// MhlTxDrvProcessDisconnection
//
///////////////////////////////////////////////////////////////////////////
static void MhlTxDrvProcessDisconnection (void)
{

//	TX_DEBUG_PRINT (("[%d] Drv: MhlTxDrvProcessDisconnection\n", (int) (HalTimerElapsed( ELAPSED_TIMER ) * MONITORING_PERIOD)));

	// clear all interrupts
	SiiRegWrite(REG_INTR4, SiiRegRead(REG_INTR4));

	SiiRegWrite(REG_MHLTX_CTL1, 0xD0);

    
	dsHpdStatus &= ~BIT6;  //cable disconnect implies downstream HPD low
	SiiRegWrite(TX_PAGE_CBUS | 0x000D, dsHpdStatus);
	SiiMhlTxNotifyDsHpdChange(0);

	if( POWER_STATE_D0_MHL == fwPowerState )
	{
		// Notify upper layer of cable removal
		SiiMhlTxNotifyConnection(false);
	}

	// Now put chip in sleep mode
	SwitchToD3();
}

///////////////////////////////////////////////////////////////////////////
//
// CbusReset
//
///////////////////////////////////////////////////////////////////////////
void CbusReset (void)
{
	uint8_t idx;
	SET_BIT(REG_SRST, 3);
	HalTimerWait(2);
	CLR_BIT(REG_SRST, 3);

	mscCmdInProgress = false;

	// Adjust interrupt mask everytime reset is performed.
    UNMASK_INTR_1_INTERRUPTS;
	UNMASK_INTR_4_INTERRUPTS;
    if (g_chipRevId < 1)
    {
		UNMASK_INTR_5_INTERRUPTS;
	}
	else
	{
		//RG disabled due to auto FIFO reset
	    MASK_INTR_5_INTERRUPTS;
	}

	UNMASK_CBUS1_INTERRUPTS;
	UNMASK_CBUS2_INTERRUPTS;

	for(idx=0; idx < 4; idx++)
	{
		// Enable WRITE_STAT interrupt for writes to all 4 MSC Status registers.
		WriteByteCBUS((0xE0 + idx), 0xFF);

		// Enable SET_INT interrupt for writes to all 4 MSC Interrupt registers.
		WriteByteCBUS((0xF0 + idx), 0xFF);
	}
}
///////////////////////////////////////////////////////////////////////////
//
// CBusProcessErrors
//
//
///////////////////////////////////////////////////////////////////////////
static uint8_t CBusProcessErrors (uint8_t intStatus)
{
    uint8_t result          = 0;
    uint8_t mscAbortReason  = 0;
	uint8_t ddcAbortReason  = 0;

    /* At this point, we only need to look at the abort interrupts. */

    intStatus &= (BIT_MSC_ABORT | BIT_MSC_XFR_ABORT);

    if ( intStatus )
    {
//      result = ERROR_CBUS_ABORT;		// No Retry will help

        /* If transfer abort or MSC abort, clear the abort reason register. */
		if( intStatus & BIT_DDC_ABORT )
		{
			result = ddcAbortReason = SiiRegRead(TX_PAGE_CBUS | REG_DDC_ABORT_REASON);
			TX_DEBUG_PRINT( ("CBUS DDC ABORT happened, reason:: %02X\n", (int)(ddcAbortReason)));
		}

        if ( intStatus & BIT_MSC_XFR_ABORT )
        {
            result = mscAbortReason = SiiRegRead(TX_PAGE_CBUS | REG_PRI_XFR_ABORT_REASON);

            TX_DEBUG_PRINT( ("CBUS:: MSC Transfer ABORTED. Clearing 0x0D\n"));
            SiiRegWrite(TX_PAGE_CBUS | REG_PRI_XFR_ABORT_REASON, 0xFF);
        }
        if ( intStatus & BIT_MSC_ABORT )
        {
            TX_DEBUG_PRINT( ("CBUS:: MSC Peer sent an ABORT. Clearing 0x0E\n"));
            SiiRegWrite(TX_PAGE_CBUS | REG_CBUS_PRI_FWR_ABORT_REASON, 0xFF);
        }

        // Now display the abort reason.

        if ( mscAbortReason != 0 )
        {
            TX_DEBUG_PRINT( ("CBUS:: Reason for ABORT is ....0x%02X = ", (int)mscAbortReason ));

            if ( mscAbortReason & CBUSABORT_BIT_REQ_MAXFAIL)
            {
                TX_DEBUG_PRINT( ("Requestor MAXFAIL - retry threshold exceeded\n"));
            }
            if ( mscAbortReason & CBUSABORT_BIT_PROTOCOL_ERROR)
            {
                TX_DEBUG_PRINT( ("Protocol Error\n"));
            }
            if ( mscAbortReason & CBUSABORT_BIT_REQ_TIMEOUT)
            {
                TX_DEBUG_PRINT( ("Requestor translation layer timeout\n"));
            }
            if ( mscAbortReason & CBUSABORT_BIT_PEER_ABORTED)
            {
                TX_DEBUG_PRINT( ("Peer sent an abort\n"));
            }
            if ( mscAbortReason & CBUSABORT_BIT_UNDEFINED_OPCODE)
            {
                TX_DEBUG_PRINT( ("Undefined opcode\n"));
            }
        }
    }
    return( result );
}

void SiiMhlTxDrvGetScratchPad (uint8_t startReg,uint8_t *pData,uint8_t length)
{
int i;
    for (i = 0; i < length;++i,++startReg)
    {
        *pData++ = SiiRegRead(TX_PAGE_CBUS | (0xC0 + startReg));
    }
}
///////////////////////////////////////////////////////////////////////////
//
// MhlCbusIsr
//
// Only when MHL connection has been established. This is where we have the
// first looks on the CBUS incoming commands or returned data bytes for the
// previous outgoing command.
//
// It simply stores the event and allows application to pick up the event
// and respond at leisure.
//
// Look for interrupts on CBUS:CBUS_INTR_STATUS [0xC8:0x08]
//		7 = RSVD			(reserved)
//		6 = MSC_RESP_ABORT	(interested)
//		5 = MSC_REQ_ABORT	(interested)	
//		4 = MSC_REQ_DONE	(interested)
//		3 = MSC_MSG_RCVD	(interested)
//		2 = DDC_ABORT		(interested)
//		1 = RSVD			(reserved)
//		0 = rsvd			(reserved)
///////////////////////////////////////////////////////////////////////////
static void MhlCbusIsr (void)
{
	uint8_t		cbusInt;
	uint8_t     gotData[4];	// Max four status and int registers.
	uint8_t		i;

	//
	// Main CBUS interrupts on CBUS_INTR_STATUS
	//
	cbusInt = SiiRegRead(TX_PAGE_CBUS | 0x0008);

	// When I2C is inoperational (D3) and a previous interrupt brought us here, do nothing.
	if(cbusInt == 0xFF)
	{
		return;
	}
	
	if( cbusInt )
	{
		TX_DEBUG_PRINT(("Drv: CBUS INTR_1: 0x%02x\n", (int) cbusInt));
	}
	
	if(cbusInt)
	{
		//
		// Clear all interrupts that were raised even if we did not process
		//
		SiiRegWrite(TX_PAGE_CBUS | 0x0008, cbusInt);
	}
	// Look for DDC_ABORT
	if (cbusInt & BIT2)
	{
		//ApplyDdcAbortSafety();
	}

	// MSC_MSG (RCP/RAP)
	if ((cbusInt & BIT3))
	{
    uint8_t mscMsg[2];
	    TX_DEBUG_PRINT(("Drv: MSC_MSG Received\n"));
		//
		// Two bytes arrive at registers 0x18 and 0x19
		//
        mscMsg[0] = SiiRegRead(TX_PAGE_CBUS | 0x0018);
        mscMsg[1] = SiiRegRead(TX_PAGE_CBUS | 0x0019);

	    TX_DEBUG_PRINT(("Drv: MSC MSG: %02X %02X\n", (int)mscMsg[0], (int)mscMsg[1] ));
		SiiMhlTxGotMhlMscMsg( mscMsg[0], mscMsg[1] );
	}
	if (cbusInt & (BIT_MSC_ABORT | BIT_MSC_XFR_ABORT | BIT_DDC_ABORT))
	{
		gotData[0] = CBusProcessErrors(cbusInt);
	}
	

	// MSC_REQ_DONE received.
	if(cbusInt & BIT4)
	{
		TX_DEBUG_PRINT(("Drv: MSC_REQ_DONE\n"));

		mscCmdInProgress = false;

		SiiMhlTxMscCommandDone( SiiRegRead(TX_PAGE_CBUS | 0x0016) );
	}
	//
	// Clear all interrupts that were raised even if we did not process
	//

	//
	// Now look for interrupts on register 0x1E. CBUS_MSC_INT2
	// 7:4 = Reserved
	//	 3 = msc_mr_write_state = We got a WRITE_STAT
	//	 2 = msc_mr_set_int. We got a SET_INT
	//	 1 = reserved
	//	 0 = msc_mr_write_burst. We received WRITE_BURST
	//
	cbusInt = SiiRegRead(TX_PAGE_CBUS | 0x001E);
	if( cbusInt )
	{
		//
		// Clear all interrupts that were raised even if we did not process
		//
		SiiRegWrite(TX_PAGE_CBUS | 0x001E, cbusInt);

	    TX_DEBUG_PRINT(("Drv: Clear CBUS INTR_2: %02X\n", (int) cbusInt));
	}

    if ( BIT0 & cbusInt)
    {
        // WRITE_BURST complete
        SiiMhlTxMscWriteBurstDone( cbusInt );
    }

	if(cbusInt & BIT2)
	{
    uint8_t intr[4];
    uint16_t address;

	    TX_DEBUG_PRINT(("Drv: MHL INTR Received\n"));

   		for(i = 0,address=0x00A0; i < 4; ++i,++address)
		{
			// Clear all, recording as we go
            intr[i] = SiiRegRead(TX_PAGE_CBUS | address);
			SiiRegWrite( (TX_PAGE_CBUS | address) , intr[i] );
		}
		// We are interested only in first two bytes.
		SiiMhlTxGotMhlIntr( intr[0], intr[1] );
	}
	if(cbusInt & BIT3)
	{
		uint8_t status[4];

		TX_DEBUG_PRINT(("Drv: MHL STATUS Received\n"));
		for(i = 0; i < 4; ++i)
		{
			// record each value
			status[i] = SiiRegRead(TX_PAGE_CBUS | (0xB0 + i));
			// clear as we go
			SiiRegWrite( (TX_PAGE_CBUS | (0x00B0 + i)), 0xFF /* future status[i]*/ );
		}

		SiiMhlTxGotMhlStatus( status[0], status[1] );
		
	}

}

static void ProcessScdtStatusChange(void)
{
	uint8_t scdtStatus;
	uint8_t mhlFifoStatus;

	scdtStatus = SiiRegRead(REG_TMDS_CSTAT);

	TX_DEBUG_PRINT(("Drv: ProcessScdtStatusChange scdtStatus: 0x%02x\n", scdtStatus));

	if (scdtStatus & 0x02)
	{
		 mhlFifoStatus = SiiRegRead(REG_INTR5);
		 TX_DEBUG_PRINT(("MHL FIFO status: 0x%02x\n", mhlFifoStatus));
		 if (mhlFifoStatus & 0x0C)
		 {
			SiiRegWrite(REG_INTR5, 0x0C);

			TX_DEBUG_PRINT(("** Apply MHL FIFO Reset\n"));
			SiiRegWrite(REG_SRST, 0x94);
			SiiRegWrite(REG_SRST, 0x84);
		 }
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvPowBitChange
//
// Alert the driver that the peer's POW bit has changed so that it can take 
// action if necessary.
//
void SiiMhlTxDrvPowBitChange (bool_t enable)
{
	// MHL peer device has it's own power
	if (enable)
	{
		SiiRegModify(REG_DISC_CTRL8, 0x04, 0x04);
	    TX_DEBUG_PRINT(("Drv: POW bit 0->1, set DISC_CTRL8[2] = 1\n"));
	}

#if (VBUS_POWER_CHK == ENABLE)
		if( vbusPowerState != enable)
		{
			vbusPowerState = enable;
			AppVbusControl( vbusPowerState );
		}
#endif
}

#if (VBUS_POWER_CHK == ENABLE)

///////////////////////////////////////////////////////////////////////////////
//
// Function Name: MHLSinkOrDonglePowerStatusCheck()
//
// Function Description: Check MHL device is dongle or sink to set inputting current limitation.
//
void MHLSinkOrDonglePowerStatusCheck (void)
{
	uint8_t RegValue;

	if( POWER_STATE_D0_MHL == fwPowerState )
	{
		SiiRegWrite( REG_CBUS_PRI_ADDR_CMD, MHL_DEV_CATEGORY_OFFSET ); 	// DevCap 0x02
		SiiRegWrite( REG_CBUS_PRI_START, MSC_START_BIT_READ_REG ); // execute DevCap reg read command

		RegValue = SiiRegRead( REG_CBUS_PRI_RD_DATA_1ST );
		TX_DEBUG_PRINT(("[MHL]: Device Category register=0x%02X...\n", (int)RegValue));
	
		if( MHL_DEV_CAT_UNPOWERED_DONGLE == (RegValue & 0x0F) )
			{
				TX_DEBUG_PRINT(("[MHL]: DevTypeValue=0x%02X, the peer is a dongle, please limit the VBUS current input from dongle to be 100mA...\n", (int)RegValue));
			}
		else if( MHL_DEV_CAT_SINGLE_INPUT_SINK == (RegValue & 0x0F) )
			{
				TX_DEBUG_PRINT(("[MHL]: DevTypeValue=0x%02X, the peer is a sink, limit the VBUS current input from sink to be 500mA...\n", (int)RegValue));
			}
	}
}
#endif

