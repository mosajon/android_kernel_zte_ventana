/*********************************************************************************/
/*  Copyright (c) 2002-2011, Silicon Image, Inc.  All rights reserved.           */
/*  No part of this work may be reproduced, modified, distributed, transmitted,  */
/*  transcribed, or translated into any language or computer format, in any form */
/*  or by any means without written permission of: Silicon Image, Inc.,          */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                         */
/*********************************************************************************/


// Standard C Library
#ifdef __KERNEL__
//#include	"sii_hal.h"
#else
//#include <stdio.h>
#endif
//#include "si_common.h"
#include "si_mhl_defs.h"
#include "si_cra.h"
#include "si_mhl_tx_api.h"
#include "si_mhl_tx.h"
#include "si_drv_mhl_tx.h"
#include "MHL_SiI8334.h"

/*
queue implementation
*/
#define NUM_CBUS_EVENT_QUEUE_EVENTS 5
typedef struct _CBusQueue_t
{
    uint8_t head;   // queue empty condition head == tail
    uint8_t tail;
    cbus_req_t queue[NUM_CBUS_EVENT_QUEUE_EVENTS];
}CBusQueue_t,*PCBusQueue_t;


#define QUEUE_SIZE(x) (sizeof(x.queue)/sizeof(x.queue[0]))
#define MAX_QUEUE_DEPTH(x) (QUEUE_SIZE(x) -1)
#define QUEUE_DEPTH(x) ((x.head <= x.tail)?(x.tail-x.head):(QUEUE_SIZE(x)-x.head+x.tail))
#define QUEUE_FULL(x) (QUEUE_DEPTH(x) >= MAX_QUEUE_DEPTH(x))

#define ADVANCE_QUEUE_HEAD(x) { x.head = (x.head < MAX_QUEUE_DEPTH(x))?(x.head+1):0; }
#define ADVANCE_QUEUE_TAIL(x) { x.tail = (x.tail < MAX_QUEUE_DEPTH(x))?(x.tail+1):0; }

#define RETREAT_QUEUE_HEAD(x) { x.head = (x.head > 0)?(x.head-1):MAX_QUEUE_DEPTH(x); }


// Because the Linux driver can be opened multiple times it can't
// depend on one time structure initialization done by the compiler.
//CBusQueue_t CBusQueue={0,0,{0}};
CBusQueue_t CBusQueue;

#ifdef __KERNEL__
#else
#pragma disable
#endif
cbus_req_t *GetNextCBusTransactionImpl(void)
{
    if (0==QUEUE_DEPTH(CBusQueue))
    {
        return NULL;
    }
    else
    {
    cbus_req_t *retVal;
        retVal = &CBusQueue.queue[CBusQueue.head];
        ADVANCE_QUEUE_HEAD(CBusQueue)
        return retVal;
    }
}
cbus_req_t *GetNextCBusTransactionWrapper(char *pszFunction,int iLine)
{
    TX_DEBUG_PRINT(("MhlTx:%d GetNextCBusTransaction: %s depth: %d  head: %d  tail: %d\n",
    				iLine,pszFunction,
                    (int)QUEUE_DEPTH(CBusQueue),
                    (int)CBusQueue.head,
                    (int)CBusQueue.tail));
    return  GetNextCBusTransactionImpl();
}
#define GetNextCBusTransaction(func) GetNextCBusTransactionWrapper(#func,__LINE__)

#ifdef __KERNEL__
#else
// pragma below disables and restores interrupts during the function that follows
#pragma disable
#endif
bool_t PutNextCBusTransactionImpl(cbus_req_t *pReq)
{
    if (QUEUE_FULL(CBusQueue))
    {
        //queue is full
        return false;
    }
    // at least one slot available
    CBusQueue.queue[CBusQueue.tail] = *pReq;
    ADVANCE_QUEUE_TAIL(CBusQueue)
    return true;
}
// use this wrapper to do debugging output for the routine above.
bool_t PutNextCBusTransactionWrapper(cbus_req_t *pReq,int iLine)
{
bool_t retVal;

    TX_DEBUG_PRINT(("MhlTx:%d PutNextCBusTransaction %02X %02X %02X depth:%d head: %d tail:%d\n"
                ,iLine
                ,(int)pReq->command
                ,(int)((MHL_MSC_MSG == pReq->command)?pReq->payload_u.msgData[0]:pReq->offsetData)
                ,(int)((MHL_MSC_MSG == pReq->command)?pReq->payload_u.msgData[1]:pReq->payload_u.msgData[0])
                ,(int)QUEUE_DEPTH(CBusQueue)
                ,(int)CBusQueue.head
                ,(int)CBusQueue.tail
                ));
    retVal = PutNextCBusTransactionImpl(pReq);

    if (!retVal)
    {
        TX_DEBUG_PRINT(("MhlTx:%d PutNextCBusTransaction queue full, when adding event %02x\n"
                        , iLine, (int)pReq->command));
    }
    return retVal;
}
#define PutNextCBusTransaction(req) PutNextCBusTransactionWrapper(req,__LINE__)

#ifdef __KERNEL__
#else
// pragma below disables and restores interrupts during the function that follows
#pragma disable
#endif
bool_t PutPriorityCBusTransactionImpl(cbus_req_t *pReq)
{
    if (QUEUE_FULL(CBusQueue))
    {
        //queue is full
        return false;
    }
    // at least one slot available
    RETREAT_QUEUE_HEAD(CBusQueue)
    CBusQueue.queue[CBusQueue.head] = *pReq;
    return true;
}
bool_t PutPriorityCBusTransactionWrapper(cbus_req_t *pReq,int iLine)
{
bool_t retVal;
    TX_DEBUG_PRINT(("MhlTx:%d: PutPriorityCBusTransaction %02X %02X %02X depth:%d head: %d tail:%d\n"
                ,iLine
                ,(int)pReq->command
                ,(int)((MHL_MSC_MSG == pReq->command)?pReq->payload_u.msgData[0]:pReq->offsetData)
                ,(int)((MHL_MSC_MSG == pReq->command)?pReq->payload_u.msgData[1]:pReq->payload_u.msgData[0])
                ,(int)QUEUE_DEPTH(CBusQueue)
                ,(int)CBusQueue.head
                ,(int)CBusQueue.tail
                ));
    retVal = PutPriorityCBusTransactionImpl(pReq);
    if (!retVal)
    {
        TX_DEBUG_PRINT(("MhlTx:%d: PutPriorityCBusTransaction queue full, when adding event 0x%02X\n",iLine,(int)pReq->command));
    }
    return retVal;
}
#define PutPriorityCBusTransaction(pReq) PutPriorityCBusTransactionWrapper(pReq,__LINE__)

#define IncrementCBusReferenceCount(func) {mhlTxConfig.cbusReferenceCount++; TX_DEBUG_PRINT(("MhlTx:%d %s cbusReferenceCount:%d\n",(int)__LINE__,#func,(int)mhlTxConfig.cbusReferenceCount)); }
#define DecrementCBusReferenceCount(func) {mhlTxConfig.cbusReferenceCount--; TX_DEBUG_PRINT(("MhlTx:%d %s cbusReferenceCount:%d\n",(int)__LINE__,#func,(int)mhlTxConfig.cbusReferenceCount)); }

#define SetMiscFlag(func,x) { mhlTxConfig.miscFlags |=  (x); TX_DEBUG_PRINT(("MhlTx:%d %s set %s\n",(int)__LINE__,#func,#x)); }
#define ClrMiscFlag(func,x) { mhlTxConfig.miscFlags &= ~(x); TX_DEBUG_PRINT(("MhlTx:%d %s clr %s\n",(int)__LINE__,#func,#x)); }
//
// Store global config info here. This is shared by the driver.
//
//
//
// structure to hold operating information of MhlTx component
//
mhlTx_config_t	mhlTxConfig={0};
//
// Functions used internally.
//
static bool_t SiiMhlTxSetDCapRdy( void );
//static bool_t SiiMhlTxClrDCapRdy( void );
static	bool_t 		SiiMhlTxRapkSend( void );

static	void		MhlTxResetStates( void );
static	bool_t		MhlTxSendMscMsg ( uint8_t command, uint8_t cmdData );

#ifdef __KERNEL__
extern 	uint8_t	   rcpSupportTable [];
#else
extern ROM uint8_t rcpSupportTable [];
#endif

bool_t MhlTxCBusBusy(void)
{
    return ((QUEUE_DEPTH(CBusQueue) > 0)||SiiMhlTxDrvCBusBusy() || mhlTxConfig.cbusReferenceCount)?true:false;
}
///////////////////////////////////////////////////////////////////////////////
// SiiMhlTxTmdsEnable
//
// Implements conditions on enabling TMDS output stated in MHL spec section 7.6.1
//
//
static void SiiMhlTxTmdsEnable(void)
{

	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxTmdsEnable\n",(int)__LINE__));
    if (MHL_RSEN & mhlTxConfig.mhlHpdRSENflags)
    {
    	TX_DEBUG_PRINT( ("\tMHL_RSEN\n"));
        if (MHL_HPD & mhlTxConfig.mhlHpdRSENflags)
        {
        	TX_DEBUG_PRINT( ("\t\tMHL_HPD\n"));
            if ((MHL_STATUS_PATH_ENABLED & mhlTxConfig.status_1)||((int) SiiRegRead((TX_PAGE_CBUS | 0x00B1))&MHL_STATUS_PATH_ENABLED))
            {
            	TX_DEBUG_PRINT(("\t\t\tMHL_STATUS_PATH_ENABLED\n"));
                SiiMhlTxDrvTmdsControl( true );
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxSetInt
//
// Set MHL defined INTERRUPT bits in peer's register set.
//
// This function returns true if operation was successfully performed.
//
//  regToWrite      Remote interrupt register to write
//
//  mask            the bits to write to that register
//
//  priority        0:  add to head of CBusQueue
//                  1:  add to tail of CBusQueue
//
static bool_t SiiMhlTxSetInt( uint8_t regToWrite,uint8_t  mask, uint8_t priorityLevel )
{
	cbus_req_t	req;
bool_t retVal;

	// find the offset and bit position
	// and feed
    req.retryCount  = 2;
	req.command     = MHL_SET_INT;
	req.offsetData  = regToWrite;
	req.payload_u.msgData[0]  = mask;
    if (0 == priorityLevel)
    {
        retVal = PutPriorityCBusTransaction(&req);
    }
    else
    {
        retVal = PutNextCBusTransaction(&req);
    }
    return retVal;
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDoWriteBurst
//
static bool_t SiiMhlTxDoWriteBurst( uint8_t startReg, uint8_t *pData,uint8_t length )
{
    if (FLAGS_WRITE_BURST_PENDING & mhlTxConfig.miscFlags)
    {
	cbus_req_t	req;
        bool_t retVal;

    	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxDoWriteBurst startReg:%d length:%d\n",(int)__LINE__,(int)startReg,(int)length) );

        req.retryCount  = 1;
    	req.command     = MHL_WRITE_BURST;
        req.length      = length;
    	req.offsetData  = startReg;
    	req.payload_u.pdatabytes  = pData;

        retVal = PutPriorityCBusTransaction(&req);
        ClrMiscFlag(MhlTxDriveStates, FLAGS_WRITE_BURST_PENDING)
        return retVal;
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////
// SiiMhlTxRequestWriteBurst
//
bool_t SiiMhlTxRequestWriteBurst(void)
{
    bool_t retVal = false;

    if (
        (FLAGS_SCRATCHPAD_BUSY & mhlTxConfig.miscFlags)
        ||
        MhlTxCBusBusy()
       )
    {
        TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxRequestWriteBurst failed FLAGS_SCRATCHPAD_BUSY \n",(int)__LINE__) );
    }
    else
    {
    	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxRequestWriteBurst, request sent\n",(int)__LINE__) );
        retVal =  SiiMhlTxSetInt(MHL_RCHANGE_INT,MHL_INT_REQ_WRT, 1);
    }

	return retVal;
}

///////////////////////////////////////////////////////////////////////////////
// SiiMhlTxInitialize
//
// Sets the transmitter component firmware up for operation, brings up chip
// into power on state first and then back to reduced-power mode D3 to conserve
// power until an MHL cable connection has been established. If the MHL port is
// used for USB operation, the chip and firmware continue to stay in D3 mode.
// Only a small circuit in the chip observes the impedance variations to see if
// processor should be interrupted to continue MHL discovery process or not.
//
// interruptDriven		If true, MhlTx component will not look at its status
//						registers in a polled manner from timer handler
//						(SiiMhlTxGetEvents). It will expect that all device
//						events will result in call to the function
//						SiiMhlTxDeviceIsr() by host's hardware or software
//						(a master interrupt handler in host software can call
//						it directly). interruptDriven == true also implies that
//						the MhlTx component shall make use of AppDisableInterrupts()
//						and AppRestoreInterrupts() for any critical section work to
//						prevent concurrency issues.
//
//						When interruptDriven == false, MhlTx component will do
//						all chip status analysis via looking at its register
//						when called periodically into the function
//						SiiMhlTxGetEvents() described below.
//
// pollIntervalMs		This number should be higher than 0 and lower than
//						51 milliseconds for effective operation of the firmware.
//						A higher number will only imply a slower response to an
//						event on MHL side which can lead to violation of a
//						connection disconnection related timing or a slower
//						response to RCP messages.
//
//
//
//
void SiiMhlTxInitialize(void)
{
	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxInitialize\n",(int)__LINE__) );

	// Initialize queue of pending CBUS requests.
	CBusQueue.head = 0;
	CBusQueue.tail = 0;

    TX_DEBUG_PRINT(("MhlTx:%d HPD: %d RSEN: %d\n"
            ,(int)__LINE__
            ,(int)((mhlTxConfig.mhlHpdRSENflags & MHL_HPD)?1:0)
            ,(int)((mhlTxConfig.mhlHpdRSENflags & MHL_RSEN)?1:0)
            ));
	MhlTxResetStates( );
    TX_DEBUG_PRINT(("MhlTx:%d HPD: %d RSEN: %d\n"
            ,(int)__LINE__
            ,(mhlTxConfig.mhlHpdRSENflags & MHL_HPD)?1:0
            ,(mhlTxConfig.mhlHpdRSENflags & MHL_RSEN)?1:0
            ));

	SiiMhlTxChipInitialize ();
}


///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGetEvents
//
// This is a function in MhlTx that must be called by application in a periodic
// fashion. The accuracy of frequency (adherence to the parameter pollIntervalMs)
// will determine adherence to some timings in the MHL specifications, however,
// MhlTx component keeps a tolerance of up to 50 milliseconds for most of the
// timings and deploys interrupt disabled mode of operation (applicable only to
// Sii 9244) for creating precise pulse of smaller duration such as 20 ms.
//
// This function does not return anything but it does modify the contents of the
// two pointers passed as parameter.
//
// It is advantageous for application to call this function in task context so
// that interrupt nesting or concurrency issues do not arise. In addition, by
// collecting the events in the same periodic polling mechanism prevents a call
// back from the MhlTx which can result in sending yet another MHL message.
//
// An example of this is responding back to an RCP message by another message
// such as RCPK or RCPE.
//
//
// *event		MhlTx returns a value in this field when function completes execution.
// 				If this field is 0, the next parameter is undefined.
//				The following values may be returned.
//
//
void SiiMhlTxGetEvents( uint8_t *event, uint8_t *eventParameter )
{
	//
	// If interrupts have not been routed to our ISR, manually call it here.
	//
	
	if(false == mhlTxConfig.interruptDriven) 
	{
		SiiMhlTxDeviceIsr();
	}
	
	MhlTxDriveStates( );

	*event = MHL_TX_EVENT_NONE;
	*eventParameter = 0;

	if( mhlTxConfig.mhlConnectionEvent )
	{
		TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxGetEvents mhlConnectionEvent\n",(int)__LINE__) );

		// Consume the message
		mhlTxConfig.mhlConnectionEvent = false;

		//
		// Let app know the connection went away.
		//
		*event          = mhlTxConfig.mhlConnected;
		*eventParameter	= mhlTxConfig.mscFeatureFlag;

		// If connection has been lost, reset all state flags.
		if(MHL_TX_EVENT_DISCONNECTION == mhlTxConfig.mhlConnected)
		{
			MhlTxResetStates( );
		}
        else if (MHL_TX_EVENT_CONNECTION == mhlTxConfig.mhlConnected)
        {
            SiiMhlTxSetDCapRdy();
        }
	}
	else if( mhlTxConfig.mscMsgArrived )
	{
		TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxGetEvents MSC MSG <%02X, %02X>\n"
                            ,(int)__LINE__
		                    ,(int) ( mhlTxConfig.mscMsgSubCommand )
							,(int) ( mhlTxConfig.mscMsgData ))
							);

		// Consume the message
		mhlTxConfig.mscMsgArrived = false;

		//
		// Map sub-command to an event id
		//
		switch( mhlTxConfig.mscMsgSubCommand )
		{
			case	MHL_MSC_MSG_RAP:
				// RAP is fully handled here.
				//
				// Handle RAP sub-commands here itself
				//
				if( MHL_RAP_CONTENT_ON == mhlTxConfig.mscMsgData)
				{
                    SiiMhlTxTmdsEnable();
				}
				else if( MHL_RAP_CONTENT_OFF == mhlTxConfig.mscMsgData)
				{
					SiiMhlTxDrvTmdsControl( false );
				}
				// Always RAPK to the peer
				SiiMhlTxRapkSend( );
				break;

			case	MHL_MSC_MSG_RCP:
				// If we get a RCP key that we do NOT support, send back RCPE
				// Do not notify app layer.
				if(MHL_LOGICAL_DEVICE_MAP & rcpSupportTable [mhlTxConfig.mscMsgData & 0x7F] )
				{
					*event          = MHL_TX_EVENT_RCP_RECEIVED;
					*eventParameter = mhlTxConfig.mscMsgData; // key code
				}
				else
				{
					// Save keycode to send a RCPK after RCPE.
					mhlTxConfig.mscSaveRcpKeyCode = mhlTxConfig.mscMsgData; // key code
					SiiMhlTxRcpeSend( RCPE_INEEFECTIVE_KEY_CODE );
				}
				break;

			case	MHL_MSC_MSG_RCPK:
				*event = MHL_TX_EVENT_RCPK_RECEIVED;
				*eventParameter = mhlTxConfig.mscMsgData; // key code
                DecrementCBusReferenceCount(SiiMhlTxGetEvents)
                mhlTxConfig.mscLastCommand = 0;
                mhlTxConfig.mscMsgLastCommand = 0;

        	    TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxGetEvents RCPK\n",(int)__LINE__) );
				break;

			case	MHL_MSC_MSG_RCPE:
				*event = MHL_TX_EVENT_RCPE_RECEIVED;
				*eventParameter = mhlTxConfig.mscMsgData; // status code
                
				break;

			case	MHL_MSC_MSG_RAPK:
				// Do nothing if RAPK comes, except decrement the reference counter
                DecrementCBusReferenceCount(SiiMhlTxGetEvents)
                mhlTxConfig.mscLastCommand = 0;
                mhlTxConfig.mscMsgLastCommand = 0;
        	    TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxGetEvents RAPK\n",(int)__LINE__) );
				break;

			default:
				// Any freak value here would continue with no event to app
				break;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// MhlTxDriveStates
//
// This is an internal function to move the MSC engine to do the next thing
// before allowing the application to run RCP APIs.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
//#define TX_DEBUG_PRINT_CASE(case) TX_DEBUG_PRINT( ("MhlTx:%d MhlTxDriveStates %s\n",(int)__LINE__, #case ) );
void	MhlTxDriveStates( void )
{
	//TX_DEBUG_PRINT( ("MhlTx:%d MhlTxDriveStates ,QUEUE_DEPTH(CBusQueue)=%d\n",(int)__LINE__, (int)QUEUE_DEPTH(CBusQueue)) );
    // process queued CBus transactions
    if (QUEUE_DEPTH(CBusQueue) > 0)
    {
        if (!SiiMhlTxDrvCBusBusy())
        {
        int reQueueRequest = 0;
        cbus_req_t *pReq = GetNextCBusTransaction(MhlTxDriveStates);
            // coordinate write burst requests and grants.
            if (MHL_SET_INT == pReq->command)
            {
                if (MHL_RCHANGE_INT == pReq->offsetData)
                {
                    if (FLAGS_SCRATCHPAD_BUSY & mhlTxConfig.miscFlags)
                    {
                        if (MHL_INT_REQ_WRT == pReq->payload_u.msgData[0])
                        {
                            reQueueRequest= 1;
                        }
                        else if (MHL_INT_GRT_WRT == pReq->payload_u.msgData[0])
                        {
                            reQueueRequest= 0;
                        }
                    }
                    else
                    {
                        if (MHL_INT_REQ_WRT == pReq->payload_u.msgData[0])
                        {
                            IncrementCBusReferenceCount(MhlTxDriveStates)
                            SetMiscFlag(MhlTxDriveStates, FLAGS_SCRATCHPAD_BUSY)
                            SetMiscFlag(MhlTxDriveStates, FLAGS_WRITE_BURST_PENDING)
                        }
                        else if (MHL_INT_GRT_WRT == pReq->payload_u.msgData[0])
                        {
                            SetMiscFlag(MhlTxDriveStates, FLAGS_SCRATCHPAD_BUSY)
                        }
                    }
                }
            }
            if (reQueueRequest)
            {
                // send this one to the back of the line for later attempts
                if (pReq->retryCount-- > 0)
                {
                    PutNextCBusTransaction(pReq);
                }
            }
            else
            {
                if (MHL_MSC_MSG == pReq->command)
                {
                    mhlTxConfig.mscMsgLastCommand = pReq->payload_u.msgData[0];
                    mhlTxConfig.mscMsgLastData    = pReq->payload_u.msgData[1];
                }
                else
                {
                    mhlTxConfig.mscLastOffset  = pReq->offsetData;
                    mhlTxConfig.mscLastData    = pReq->payload_u.msgData[0];

                }
                mhlTxConfig.mscLastCommand = pReq->command;

                IncrementCBusReferenceCount(MhlTxDriveStates)
                SiiMhlTxDrvSendCbusCommand( pReq  );
            }
        }
    }

}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxMscCommandDone
//
// This function is called by the driver to inform of completion of last command.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
#define FLAG_OR_NOT(x) (FLAGS_HAVE_##x & mhlTxConfig.miscFlags)?#x:""
#define SENT_OR_NOT(x) (FLAGS_SENT_##x & mhlTxConfig.miscFlags)?#x:""

void	SiiMhlTxMscCommandDone( uint8_t data1 )
{
	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxMscCommandDone. data1 = %02X\n",(int)__LINE__, (int) data1) );

    DecrementCBusReferenceCount(SiiMhlTxMscCommandDone)
    if ( MHL_READ_DEVCAP == mhlTxConfig.mscLastCommand )
    {
    	if(MHL_DEV_CATEGORY_OFFSET == mhlTxConfig.mscLastOffset)
		{
            mhlTxConfig.miscFlags |= FLAGS_HAVE_DEV_CATEGORY;
        	TX_DEBUG_PRINT(("MhlTx:%d SiiMhlTxMscCommandDone FLAGS_HAVE_DEV_CATEGORY\n",(int)__LINE__));

			SiiMhlTxDrvPowBitChange((bool_t) ( data1 & MHL_DEV_CATEGORY_POW_BIT));

            // OK to call this here, since requests always get queued and processed in the "foreground"
			SiiMhlTxReadDevcap( MHL_DEV_FEATURE_FLAG_OFFSET );
	}
    	else if(MHL_DEV_FEATURE_FLAG_OFFSET == mhlTxConfig.mscLastOffset)
		{
            mhlTxConfig.miscFlags |= FLAGS_HAVE_DEV_FEATURE_FLAGS;
        	TX_DEBUG_PRINT(("MhlTx:%d SiiMhlTxMscCommandDone FLAGS_HAVE_DEV_FEATURE_FLAGS\n",(int)__LINE__));

			// Remember features of the peer
			mhlTxConfig.mscFeatureFlag	= data1;

			// These variables are used to remember if we issued a READ_DEVCAP
	   		//    or other MSC command
			// Since we are done, reset them.
			mhlTxConfig.mscLastCommand = 0;
			mhlTxConfig.mscLastOffset  = 0;

    		TX_DEBUG_PRINT( ("MhlTx:%d Peer's Feature Flag = %02X\n\n",(int)__LINE__, (int) data1) );
    	}
	}
	else if(MHL_WRITE_STAT == mhlTxConfig.mscLastCommand)
	{

    	TX_DEBUG_PRINT( ("MhlTx: WRITE_STAT miscFlags: %02X\n\n", (int) mhlTxConfig.miscFlags) );
        if (MHL_STATUS_REG_CONNECTED_RDY == mhlTxConfig.mscLastOffset)
        {
            if (MHL_STATUS_DCAP_RDY & mhlTxConfig.mscLastData)
            {
                mhlTxConfig.miscFlags |= FLAGS_SENT_DCAP_RDY;
            	TX_DEBUG_PRINT(("MhlTx:%d SiiMhlTxMscCommandDone FLAGS_SENT_DCAP_RDY\n",(int)__LINE__));
            }
		}
        else if (MHL_STATUS_REG_LINK_MODE == mhlTxConfig.mscLastOffset)
        {
            if ( MHL_STATUS_PATH_ENABLED & mhlTxConfig.mscLastData)
	        {
                mhlTxConfig.miscFlags |= FLAGS_SENT_PATH_EN;
            	TX_DEBUG_PRINT(("MhlTx:%d SiiMhlTxMscCommandDone FLAGS_SENT_PATH_EN\n",(int)__LINE__));
   	    	}
		}


    	mhlTxConfig.mscLastCommand = 0;
    	mhlTxConfig.mscLastOffset  = 0;
	}
	else if (MHL_MSC_MSG == mhlTxConfig.mscLastCommand)
    {
    	if(MHL_MSC_MSG_RCPE == mhlTxConfig.mscMsgLastCommand)
		{
			//
			// RCPE is always followed by an RCPK with original key code that came.
			//
			if( SiiMhlTxRcpkSend( mhlTxConfig.mscSaveRcpKeyCode ) )
			{
    		}
    	}
        else
        {
    	    TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxMscCommandDone default\n"
    	            "\tmscLastCommand: 0x%02X \n"
    	            "\tmscMsgLastCommand: 0x%02X mscMsgLastData: 0x%02X\n"
                    "\tcbusReferenceCount: %d\n"
    	            ,(int)__LINE__
    	            ,(int)mhlTxConfig.mscLastCommand
    	            ,(int)mhlTxConfig.mscMsgLastCommand
    	            ,(int)mhlTxConfig.mscMsgLastData
                    ,(int)mhlTxConfig.cbusReferenceCount
    	            ) );
        }
        mhlTxConfig.mscLastCommand = 0;
    }
    else if (MHL_WRITE_BURST == mhlTxConfig.mscLastCommand)
    {
        TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxMscCommandDone MHL_WRITE_BURST\n",(int)__LINE__ ) );
        mhlTxConfig.mscLastCommand = 0;
        mhlTxConfig.mscLastOffset  = 0;
        mhlTxConfig.mscLastData    = 0;

        // all CBus request are queued, so this is OK to call here
        // use priority 0 so that other queued commands don't interfere
        SiiMhlTxSetInt( MHL_RCHANGE_INT,MHL_INT_DSCR_CHG,0 );
    }
    else if (MHL_SET_INT == mhlTxConfig.mscLastCommand)
    {
        TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxMscCommandDone MHL_SET_INT\n",(int)__LINE__ ) );
        if (MHL_RCHANGE_INT == mhlTxConfig.mscLastOffset)
        {
        	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxMscCommandDone MHL_RCHANGE_INT\n",(int)__LINE__) );
            if (MHL_INT_DSCR_CHG == mhlTxConfig.mscLastData)
            {
                DecrementCBusReferenceCount(SiiMhlTxMscCommandDone)  // this one is for the write burst request
                TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxMscCommandDone MHL_INT_DSCR_CHG\n",(int)__LINE__) );
                ClrMiscFlag(SiiMhlTxMscCommandDone, FLAGS_SCRATCHPAD_BUSY)
            }
        }
			// Once the command has been sent out successfully, forget this case.
        mhlTxConfig.mscLastCommand = 0;
        mhlTxConfig.mscLastOffset  = 0;
        mhlTxConfig.mscLastData    = 0;
    }
    else
    {
    	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxMscCommandDone default\n"
    	            "\tmscLastCommand: 0x%02X mscLastOffset: 0x%02X\n"
                    "\tcbusReferenceCount: %d\n"
    	            ,(int)__LINE__
    	            ,(int)mhlTxConfig.mscLastCommand
    	            ,(int)mhlTxConfig.mscLastOffset
                    ,(int)mhlTxConfig.cbusReferenceCount
    	            ) );
    }
    if (!(FLAGS_RCP_READY & mhlTxConfig.miscFlags))
    {
    	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxMscCommandDone. have(%s %s) sent(%s %s)\n"
    	                    , (int) __LINE__
                            , FLAG_OR_NOT(DEV_CATEGORY)
                            , FLAG_OR_NOT(DEV_FEATURE_FLAGS)
                            , SENT_OR_NOT(PATH_EN)
                            , SENT_OR_NOT(DCAP_RDY)
    	));
        if (FLAGS_HAVE_DEV_CATEGORY & mhlTxConfig.miscFlags)
        {
            if (FLAGS_HAVE_DEV_FEATURE_FLAGS& mhlTxConfig.miscFlags)
            {
                if (FLAGS_SENT_PATH_EN & mhlTxConfig.miscFlags)
                {
                    if (FLAGS_SENT_DCAP_RDY & mhlTxConfig.miscFlags)
                    {
                        mhlTxConfig.miscFlags |= FLAGS_RCP_READY;
                		// Now we can entertain App commands for RCP
                		// Let app know this state
                		mhlTxConfig.mhlConnectionEvent = true;
                		mhlTxConfig.mhlConnected = MHL_TX_EVENT_RCP_READY;
                    }
                }
            }
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxMscWriteBurstDone
//
// This function is called by the driver to inform of completion of a write burst.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
void	SiiMhlTxMscWriteBurstDone( uint8_t data1 )
{
	#define WRITE_BURST_TEST_SIZE 16
	uint8_t temp[WRITE_BURST_TEST_SIZE];
	uint8_t i;
    TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxMscWriteBurstDone(%02X) \"",(int)__LINE__,(int)data1 ) );
    SiiMhlTxDrvGetScratchPad(0,temp,WRITE_BURST_TEST_SIZE);
    for (i = 0; i < WRITE_BURST_TEST_SIZE ; ++i)
    {
        if (temp[i]>=' ')
        {
            TX_DEBUG_PRINT(("%02X %c ",(int)temp[i],temp[i]));
        }
        else
        {
            TX_DEBUG_PRINT(("%02X . ",(int)temp[i]));
        }
    }
    TX_DEBUG_PRINT(("\"\n"));
}


///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlMscMsg
//
// This function is called by the driver to inform of arrival of a MHL MSC_MSG
// such as RCP, RCPK, RCPE. To quickly return back to interrupt, this function
// remembers the event (to be picked up by app later in task context).
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing of its own,
//
// No printfs.
//
// Application shall not call this function.
//
void	SiiMhlTxGotMhlMscMsg( uint8_t subCommand, uint8_t cmdData )
{
	// Remeber the event.
	mhlTxConfig.mscMsgArrived		= true;
	mhlTxConfig.mscMsgSubCommand	= subCommand;
	mhlTxConfig.mscMsgData			= cmdData;
}
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlIntr
//
// This function is called by the driver to inform of arrival of a MHL INTERRUPT.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
void	SiiMhlTxGotMhlIntr( uint8_t intr_0, uint8_t intr_1 )
{
	TX_DEBUG_PRINT( ("MhlTx:%d INTERRUPT Arrived. %02X, %02X\n",(int)__LINE__, (int) intr_0, (int) intr_1) );

	//
	// Handle DCAP_CHG INTR here
	//
	if((MHL_INT_DCAP_CHG & intr_0)||(MHL_INT_DCAP_CHG & intr_1))
	{
        // OK to call this here, since all requests are queued
		SiiMhlTxReadDevcap( MHL_DEV_CATEGORY_OFFSET );
	}

	if( MHL_INT_DSCR_CHG & intr_0)
    {
        SiiMhlTxDrvGetScratchPad(0,mhlTxConfig.localScratchPad,sizeof(mhlTxConfig.localScratchPad));
        // remote WRITE_BURST is complete
        ClrMiscFlag(SiiMhlTxGotMhlIntr, FLAGS_SCRATCHPAD_BUSY)
    }
	if( MHL_INT_REQ_WRT  & intr_0)
    {

        // this is a request from the sink device.
        if (FLAGS_SCRATCHPAD_BUSY & mhlTxConfig.miscFlags)
        {
            // use priority 1 to defer sending grant until
            //  local traffic is done
            SiiMhlTxSetInt( MHL_RCHANGE_INT, MHL_INT_GRT_WRT,1);
        }
        else
        {
            SetMiscFlag(SiiMhlTxGotMhlIntr, FLAGS_SCRATCHPAD_BUSY)
            // OK to call this here, since all requests are queued
            // use priority 0 to respond immediately
            SiiMhlTxSetInt( MHL_RCHANGE_INT, MHL_INT_GRT_WRT,0);
        }
    }
	if( MHL_INT_GRT_WRT  & intr_0)
    {
    	uint8_t length =sizeof(mhlTxConfig.localScratchPad);
        TX_DEBUG_PRINT(("MhlTx:%d MHL_INT_GRT_WRT length:%d\n",(int)__LINE__,(int)length));
        SiiMhlTxDoWriteBurst(0x40, mhlTxConfig.localScratchPad, length);
    }

    // removed "else", since interrupts are not mutually exclusive of each other.
	if(MHL_INT_EDID_CHG & intr_1)
	{
		// force upstream source to read the EDID again.
		// Most likely by appropriate togggling of HDMI HPD
		SiiMhlTxDrvNotifyEdidChange ( );
	}
}
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlStatus
//
// This function is called by the driver to inform of arrival of a MHL STATUS.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
void	SiiMhlTxGotMhlStatus( uint8_t status_0, uint8_t status_1 )
{
	//
	// Handle DCAP_RDY STATUS here itself
	//
	uint8_t StatusChangeBitMask0,StatusChangeBitMask1;
	TX_DEBUG_PRINT( ("MhlTx: STATUS Arrived. %02X, %02X\n", (int) status_0, (int) status_1) );
    StatusChangeBitMask0 = status_0 ^ mhlTxConfig.status_0;
    StatusChangeBitMask1 = status_1 ^ mhlTxConfig.status_1;
	// Remember the event.   (other code checks the saved values, so save the values early, but not before the XOR operations above)
	mhlTxConfig.status_0 = status_0;
	mhlTxConfig.status_1 = status_1;
	
	if(MHL_STATUS_DCAP_RDY & StatusChangeBitMask0)
	{

        TX_DEBUG_PRINT(("MhlTx: DCAP_RDY changed\n"));
        if (MHL_STATUS_DCAP_RDY & status_0)
        {
            // OK to call this here since all requests are queued
    		SiiMhlTxReadDevcap( MHL_DEV_CATEGORY_OFFSET );
        }
	}
    // did PATH_EN change?
	if(MHL_STATUS_PATH_ENABLED & StatusChangeBitMask1)
    {
        TX_DEBUG_PRINT(("MhlTx: PATH_EN changed\n"));
        if(MHL_STATUS_PATH_ENABLED & status_1)
        {
            // OK to call this here since all requests are queued
            SiiMhlTxSetPathEn();
        }
        else
        {
            // OK to call this here since all requests are queued
            SiiMhlTxClrPathEn();
        }
    }

}
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxRcpSend
//
// This function checks if the peer device supports RCP and sends rcpKeyCode. The
// function will return a value of true if it could successfully send the RCP
// subcommand and the key code. Otherwise false.
//
// The followings are not yet utilized.
//
// (MHL_FEATURE_RAP_SUPPORT & mhlTxConfig.mscFeatureFlag))
// (MHL_FEATURE_SP_SUPPORT & mhlTxConfig.mscFeatureFlag))
//
//
bool_t SiiMhlTxRcpSend( uint8_t rcpKeyCode )
{
	bool_t retVal;
	//
	// If peer does not support do not send RCP or RCPK/RCPE commands
	//

	if((0 == (MHL_FEATURE_RCP_SUPPORT & mhlTxConfig.mscFeatureFlag))
	    ||
        !(FLAGS_RCP_READY & mhlTxConfig.miscFlags)
		)
	{
    	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxRcpSend failed\n",(int)__LINE__) );
		retVal=false;
	}

	retVal=MhlTxSendMscMsg ( MHL_MSC_MSG_RCP, rcpKeyCode );
    if(retVal)
    {
    	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxRcpSend\n",(int)__LINE__) );
        IncrementCBusReferenceCount(SiiMhlTxRcpSend)
		MhlTxDriveStates();
	}
    return retVal;
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxRcpkSend
//
// This function sends RCPK to the peer device.
//
bool_t SiiMhlTxRcpkSend( uint8_t rcpKeyCode )
{
	bool_t	retVal;

	retVal = MhlTxSendMscMsg(MHL_MSC_MSG_RCPK, rcpKeyCode);
	if(retVal) {
		MhlTxDriveStates();
	}
	return retVal;
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxRapkSend
//
// This function sends RAPK to the peer device. 
//
static	bool_t SiiMhlTxRapkSend( void )
{
	return	( MhlTxSendMscMsg ( MHL_MSC_MSG_RAPK, 0 ) );
}
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxRcpeSend
//
// The function will return a value of true if it could successfully send the RCPE
// subcommand. Otherwise false.
//
// When successful, MhlTx internally sends RCPK with original (last known)
// keycode.
//
bool_t SiiMhlTxRcpeSend( uint8_t rcpeErrorCode )
{
	bool_t	retVal;

	retVal = MhlTxSendMscMsg(MHL_MSC_MSG_RCPE, rcpeErrorCode);
	if(retVal) {
		MhlTxDriveStates();
	}
	return retVal;
}

/*
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxRapSend
//
// This function checks if the peer device supports RAP and sends rcpKeyCode. The
// function will return a value of true if it could successfully send the RCP
// subcommand and the key code. Otherwise false.
//

bool_t SiiMhlTxRapSend( uint8_t rapActionCode )
{
bool_t retVal;
    if (!(FLAGS_RCP_READY & mhlTxConfig.miscFlags))
    {
    	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxRapSend failed\n",(int)__LINE__) );
        retVal = false;
    }
    else
    {
    	retVal = MhlTxSendMscMsg ( MHL_MSC_MSG_RAP, rapActionCode );
        if(retVal)
        {
            IncrementCBusReferenceCount
            TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxRapSend\n",(int)__LINE__) );
        }
    }
    return retVal;
}

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
void	SiiMhlTxGotMhlWriteBurst( uint8_t *spadArray )
{
}
*/
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxSetStatus
//
// Set MHL defined STATUS bits in peer's register set.
//
// register	    MHLRegister to write
//
// value        data to write to the register
//

static bool_t SiiMhlTxSetStatus( uint8_t regToWrite, uint8_t value )
{
	cbus_req_t	req;
    bool_t retVal;

	// find the offset and bit position
	// and feed
    req.retryCount  = 2;
	req.command     = MHL_WRITE_STAT;
	req.offsetData  = regToWrite;
	req.payload_u.msgData[0]  = value;

    TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxSetStatus\n",(int)__LINE__) );
    retVal = PutNextCBusTransaction(&req);
    return retVal;
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxSetDCapRdy
//
static bool_t SiiMhlTxSetDCapRdy( void )
{
    mhlTxConfig.connectedReady |= MHL_STATUS_DCAP_RDY;   // update local copy
    return SiiMhlTxSetStatus( MHL_STATUS_REG_CONNECTED_RDY, mhlTxConfig.connectedReady);
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxClrDCapRdy
//
#if 0
static bool_t SiiMhlTxClrDCapRdy( void )
{
    mhlTxConfig.connectedReady &= ~MHL_STATUS_DCAP_RDY;  // update local copy
    return SiiMhlTxSetStatus( MHL_STATUS_REG_CONNECTED_RDY, mhlTxConfig.connectedReady);
}
#endif
///////////////////////////////////////////////////////////////////////////////
//
//  SiiMhlTxSendLinkMode
//
static bool_t SiiMhlTxSendLinkMode(void)
{
    return SiiMhlTxSetStatus( MHL_STATUS_REG_LINK_MODE, mhlTxConfig.linkMode);
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxSetPathEn
//
bool_t SiiMhlTxSetPathEn(void )
{
	TX_DEBUG_PRINT(("MhlTx:%d SiiMhlTxSetPathEn\n",(int)__LINE__));
    SiiMhlTxTmdsEnable();
    mhlTxConfig.linkMode |= MHL_STATUS_PATH_ENABLED;     // update local copy
    return SiiMhlTxSetStatus( MHL_STATUS_REG_LINK_MODE, mhlTxConfig.linkMode);
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxClrPathEn
//
bool_t SiiMhlTxClrPathEn( void )
{
	TX_DEBUG_PRINT(("MhlTx:%d SiiMhlTxClrPathEn\n",(int)__LINE__));
    SiiMhlTxDrvTmdsControl( false );
    mhlTxConfig.linkMode &= ~MHL_STATUS_PATH_ENABLED;    // update local copy
    return SiiMhlTxSetStatus( MHL_STATUS_REG_LINK_MODE, mhlTxConfig.linkMode);
}

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
bool_t SiiMhlTxReadDevcap( uint8_t offset )
{
	cbus_req_t	req;
	TX_DEBUG_PRINT( ("MhlTx:%d SiiMhlTxReadDevcap\n",(int)__LINE__));
	//
	// Send MHL_READ_DEVCAP command
	//
    req.retryCount  = 2;
	req.command     = MHL_READ_DEVCAP;
	req.offsetData  = offset;
    req.payload_u.msgData[0]  = 0;  // do this to avoid confusion

    return PutNextCBusTransaction(&req);
}

///////////////////////////////////////////////////////////////////////////////
//
// MhlTxSendMscMsg
//
// This function sends a MSC_MSG command to the peer.
// It  returns true if successful in doing so.
//
// The value of devcap should be obtained by making a call to SiiMhlTxGetEvents()
//
// offset		Which byte in devcap register is required to be read. 0..0x0E
//
static bool_t MhlTxSendMscMsg ( uint8_t command, uint8_t cmdData )
{
	cbus_req_t	req;
	uint8_t		ccode;

	//
	// Send MSC_MSG command
	//
	// Remember last MSC_MSG command (RCPE particularly)
	//
    req.retryCount  = 2;
	req.command     = MHL_MSC_MSG;
	req.payload_u.msgData[0]  = command;
	req.payload_u.msgData[1]  = cmdData;
    ccode = PutNextCBusTransaction(&req);
	return( (bool_t) ccode );
}
///////////////////////////////////////////////////////////////////////////////
// 
// SiiMhlTxNotifyConnection
//
//
void	SiiMhlTxNotifyConnection( bool_t mhlConnected )
{
	mhlTxConfig.mhlConnectionEvent = true;

	TX_DEBUG_PRINT(("MhlTx: SiiMhlTxNotifyConnection MSC_STATE_IDLE %01X\n", (int) mhlConnected ));

	if(mhlConnected)
	{
		mhlTxConfig.mhlConnected = MHL_TX_EVENT_CONNECTION;
        mhlTxConfig.mhlHpdRSENflags |= MHL_RSEN;
        SiiMhlTxTmdsEnable();
        SiiMhlTxSendLinkMode();
	}
	else
	{
		mhlTxConfig.mhlConnected = MHL_TX_EVENT_DISCONNECTION;
        mhlTxConfig.mhlHpdRSENflags &= ~MHL_RSEN;
	}
}
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxNotifyDsHpdChange
// Driver tells about arrival of SET_HPD or CLEAR_HPD by calling this function.
//
// Turn the content off or on based on what we got.
//
void	SiiMhlTxNotifyDsHpdChange( uint8_t dsHpdStatus )
{
	if( 0 == dsHpdStatus )
	{
	    TX_DEBUG_PRINT(("MhlTx: Disable TMDS\n"));
	    TX_DEBUG_PRINT(("MhlTx: DsHPD OFF\n"));
        mhlTxConfig.mhlHpdRSENflags &= ~MHL_HPD;
		AppNotifyMhlDownStreamHPDStatusChange(dsHpdStatus);
		SiiMhlTxDrvTmdsControl( false );
	}
	else
	{
	    TX_DEBUG_PRINT(("MhlTx: Enable TMDS\n"));
	    TX_DEBUG_PRINT(("MhlTx: DsHPD ON\n"));
        mhlTxConfig.mhlHpdRSENflags |= MHL_HPD;
		AppNotifyMhlDownStreamHPDStatusChange(dsHpdStatus);
        SiiMhlTxTmdsEnable();
	}
}
///////////////////////////////////////////////////////////////////////////////
//
// MhlTxResetStates
//
// Application picks up mhl connection and rcp events at periodic intervals.
// Interrupt handler feeds these variables. Reset them on disconnection.
//
static void	MhlTxResetStates( void )
{
	mhlTxConfig.mhlConnectionEvent	= false;
	mhlTxConfig.mhlConnected		= MHL_TX_EVENT_DISCONNECTION;
    mhlTxConfig.mhlHpdRSENflags    &= ~(MHL_RSEN | MHL_HPD);
	mhlTxConfig.mscMsgArrived		= false;

    mhlTxConfig.status_0            = 0;
    mhlTxConfig.status_1            = 0;
    mhlTxConfig.connectedReady      = 0;
    mhlTxConfig.linkMode            = 3; // indicate normal (24-bit) mode
    mhlTxConfig.cbusReferenceCount  = 0;
    mhlTxConfig.miscFlags           = 0;
    mhlTxConfig.mscLastCommand      = 0;
    mhlTxConfig.mscMsgLastCommand   = 0;
}

/*
    SiiTxReadConnectionStatus
    returns:
    0: if not fully connected
    1: if fully connected
*/
uint8_t    SiiTxReadConnectionStatus(void)
{
    return (mhlTxConfig.mhlConnected >= MHL_TX_EVENT_RCP_READY)?1:0;
}

///////////////////////////////////////////////////////////////////////////////
//
// AppNotifyMhlDownStreamHPDStatusChange
//
//  This function is invoked from the MhlTx component to notify the application about
//  changes to the Downstream HPD state of the MHL subsystem.
//
// Application module must provide this function.
//
void  AppNotifyMhlDownStreamHPDStatusChange(bool_t connected)
{
	connected = connected;	// suppress warning.
}

///////////////////////////////////////////////////////////////////////////////
//
// AppNotifyMhlEvent
//
//  This function is invoked from the MhlTx component to notify the application
//  about detected events that may be of interest to it.
//
// Application module must provide this function.
//
void  AppNotifyMhlEvent(uint8_t eventCode, uint8_t eventParam)
{
}

