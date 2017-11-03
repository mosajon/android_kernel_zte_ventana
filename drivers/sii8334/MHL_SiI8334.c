/***********************************************************************************/
/* File Name: MHL_SiI8334.c */
/* File Description: this file is used to make sii8334 driver to be added in kernel or module. */

/*  Copyright (c) 2002-2010, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/kthread.h>

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <linux/gpio.h>
#include <linux/delay.h>

#include "MHL_SiI8334.h"
#include "si_mhl_tx_api.h"
#include "si_mhl_tx.h"
#include "si_drv_mhl_tx.h"
#include "si_mhl_defs.h"
//interrupt mode or polling mode for 8334 driver (if you want to use polling mode, pls comment below sentense)
#define SiI8334DRIVER_INTERRUPT_MODE   1

//Debug test
#undef dev_info
#define dev_info dev_err
#define MHL_DRIVER_NAME "sii8334drv"

#define MHL_DRIVER_MINOR_MAX   1
#define EVENT_POLL_INTERVAL_30_MS	30

/***** public type definitions ***********************************************/

typedef struct {
	struct task_struct	*pTaskStruct;
	uint8_t				pendingEvent;		// event data wait for retrieval
	uint8_t				pendingEventData;	// by user mode application

} MHL_DRIVER_CONTEXT_T, *PMHL_DRIVER_CONTEXT_T;


/***** global variables ********************************************/

MHL_DRIVER_CONTEXT_T gDriverContext;


struct platform_data {
	void (*reset) (void);
};

static struct platform_data *Sii8334_plat_data;


bool_t	vbusPowerState = true;		// false: 0 = vbus output on; true: 1 = vbus output off;

static bool_t match_id(const struct i2c_device_id *id, const struct i2c_client *client)
{
	if (strcmp(client->name, id->name) == 0)
		return true;

	return false;
}


static bool_t Sii8334_mhl_reset(void)
{
	Sii8334_plat_data = sii8334_PAGE_TPI->dev.platform_data;
	if (Sii8334_plat_data->reset){
		Sii8334_plat_data->reset();
		return true;
		}
	return false;
}

//------------------------------------------------------------------------------
// Function:    HalTimerWait
// Description: Waits for the specified number of milliseconds, using timer 0.
//------------------------------------------------------------------------------

void HalTimerWait ( uint16_t ms )
{
	msleep(ms);
}

//------------------------------------------------------------------------------
//Function:     AppRcpDemo
//Description:  This function is supposed to provide a demo code to elicit how to call RCP API function.
//------------------------------------------------------------------------------

#define	APP_DEMO_RCP_SEND_KEY_CODE 0x44

void	AppRcpDemo( uint8_t event, uint8_t eventParameter)
{
	uint8_t 	rcpKeyCode;

//	printf("App: Got event = %02X, eventParameter = %02X\n", (int)event, (int)eventParameter);

	switch( event )
	{
		case	MHL_TX_EVENT_DISCONNECTION:
			printk("App: Got event = MHL_TX_EVENT_DISCONNECTION\n");
			break;

		case	MHL_TX_EVENT_CONNECTION:
			printk("App: Got event = MHL_TX_EVENT_CONNECTION\n");
			break;

		case	MHL_TX_EVENT_RCP_READY:
		
			// Demo RCP key code PLAY
			rcpKeyCode = APP_DEMO_RCP_SEND_KEY_CODE;

			printk("App: Got event = MHL_TX_EVENT_RCP_READY...Sending RCP (%02X)\n", (int) rcpKeyCode);


			if( (0 == (MHL_FEATURE_RCP_SUPPORT & eventParameter)) )
			{
				printk( "App: Peer does NOT support RCP\n" );
			}
			if( (0 == (MHL_FEATURE_RAP_SUPPORT & eventParameter)) )
			{
				printk( "App: Peer does NOT support RAP\n" );
			}
			if( (0 == (MHL_FEATURE_SP_SUPPORT & eventParameter)) )
			{
				printk( "App: Peer does NOT support WRITE_BURST\n" );
			}


			//
			// If RCP engine is ready, send one code
			//
			if( SiiMhlTxRcpSend( rcpKeyCode ))
			{
				printk("App: SiiMhlTxRcpSend (%02X)\n", (int) rcpKeyCode);
			}
			else
			{
				printk("App: SiiMhlTxRcpSend (%02X) Returned Failure.\n", (int) rcpKeyCode);
			}

			break;

		case	MHL_TX_EVENT_RCP_RECEIVED:
			//
			// Check if we got an RCP. Application can perform the operation here
			// and send RCPK or RCPE. For now, we send the RCPK
			//
			rcpKeyCode = eventParameter;
			printk("App1: Received an RCP key code = %02X\n", (int)rcpKeyCode );

			// Added RCP key printf and interface with UI. //by oscar 20101217
				switch ( rcpKeyCode )
				{
					case MHL_RCP_CMD_SELECT:
					TX_DEBUG_PRINT(( "\nSelect received\n\n" ));
					break;
					case MHL_RCP_CMD_UP:
					TX_DEBUG_PRINT(( "\nUp received\n\n" ));
					break;
					case MHL_RCP_CMD_DOWN:
					TX_DEBUG_PRINT(( "\nDown received\n\n" ));
					break;
					case MHL_RCP_CMD_LEFT:
					TX_DEBUG_PRINT(( "\nLeft received\n\n" ));
					break;
					case MHL_RCP_CMD_RIGHT:
					TX_DEBUG_PRINT(( "\nRight received\n\n" ));
					break;
					case MHL_RCP_CMD_ROOT_MENU:
					TX_DEBUG_PRINT(( "\nRoot Menu received\n\n" ));
					break;
					case MHL_RCP_CMD_EXIT:
					TX_DEBUG_PRINT(( "\nExit received\n\n" ));
					break;
					case MHL_RCP_CMD_NUM_0:
					TX_DEBUG_PRINT(( "\nNumber 0 received\n\n" ));
					break;
					case MHL_RCP_CMD_NUM_1:
					TX_DEBUG_PRINT(( "\nNumber 1 received\n\n" ));
					break;
					case MHL_RCP_CMD_NUM_2:
					TX_DEBUG_PRINT(( "\nNumber 2 received\n\n" ));
					break;	
					case MHL_RCP_CMD_NUM_3:
					TX_DEBUG_PRINT(( "\nNumber 3 received\n\n" ));
					break;	
					case MHL_RCP_CMD_NUM_4:
					TX_DEBUG_PRINT(( "\nNumber 4 received\n\n" ));
					break;
					case MHL_RCP_CMD_NUM_5:
					TX_DEBUG_PRINT(( "\nNumber 5 received\n\n" ));
					break;	
					case MHL_RCP_CMD_NUM_6:
					TX_DEBUG_PRINT(( "\nNumber 6 received\n\n" ));
					break;
					case MHL_RCP_CMD_NUM_7:
					TX_DEBUG_PRINT(( "\nNumber 7 received\n\n" ));
					break;
					case MHL_RCP_CMD_NUM_8:
					TX_DEBUG_PRINT(( "\nNumber 8 received\n\n" ));
					break;
					case MHL_RCP_CMD_NUM_9:
					TX_DEBUG_PRINT(( "\nNumber 9 received\n\n" ));
					break;
					case MHL_RCP_CMD_DOT:
					TX_DEBUG_PRINT(( "\nDot received\n\n" ));
					break;
					case MHL_RCP_CMD_ENTER:
					TX_DEBUG_PRINT(( "\nEnter received\n\n" ));
					break;
					case MHL_RCP_CMD_CLEAR:
					TX_DEBUG_PRINT(( "\nClear received\n\n" ));
					break;
				case MHL_RCP_CMD_SOUND_SELECT:
					TX_DEBUG_PRINT(( "\nSound Select received\n\n" ));
					break;
					case MHL_RCP_CMD_PLAY:
					TX_DEBUG_PRINT(( "\nPlay received\n\n" ));
					break;
					case MHL_RCP_CMD_PAUSE:
					TX_DEBUG_PRINT(( "\nPause received\n\n" ));
					break;
					case MHL_RCP_CMD_STOP:
					TX_DEBUG_PRINT(( "\nStop received\n\n" ));
					break;
					case MHL_RCP_CMD_FAST_FWD:
					TX_DEBUG_PRINT(( "\nFastfwd received\n\n" ));
					break;
					case MHL_RCP_CMD_REWIND:
					TX_DEBUG_PRINT(( "\nRewind received\n\n" ));
					break;
				case MHL_RCP_CMD_EJECT:
					TX_DEBUG_PRINT(( "\nEject received\n\n" ));
					break;
				case MHL_RCP_CMD_FWD:
					TX_DEBUG_PRINT(( "\nForward received\n\n" ));
					break;
				case MHL_RCP_CMD_BKWD:
					TX_DEBUG_PRINT(( "\nBackward received\n\n" ));
					break;
					case MHL_RCP_CMD_PLAY_FUNC:
					TX_DEBUG_PRINT(( "\nPlay Function received\n\n" ));
					break;
					case MHL_RCP_CMD_PAUSE_PLAY_FUNC:
					TX_DEBUG_PRINT(( "\nPause_Play Function received\n\n" ));
					break;
				case MHL_RCP_CMD_STOP_FUNC:
					TX_DEBUG_PRINT(( "\nStop Function received\n\n" ));
					break;
				case MHL_RCP_CMD_F1:
					TX_DEBUG_PRINT(( "\nF1 received\n\n" ));
					break;
				case MHL_RCP_CMD_F2:
					TX_DEBUG_PRINT(( "\nF2 received\n\n" ));
					break;
				case MHL_RCP_CMD_F3:
					TX_DEBUG_PRINT(( "\nF3 received\n\n" ));
					break;
				case MHL_RCP_CMD_F4:
					TX_DEBUG_PRINT(( "\nF4 received\n\n" ));
					break;
				case MHL_RCP_CMD_F5:
					TX_DEBUG_PRINT(( "\nF5 received\n\n" ));
					break;
					default:
						break;
				}

			SiiMhlTxRcpkSend(rcpKeyCode);
			break;

		case	MHL_TX_EVENT_RCPK_RECEIVED:
			printk("App: Received an RCPK = %02X\n",MHL_TX_EVENT_RCPK_RECEIVED);
			break;

		case	MHL_TX_EVENT_RCPE_RECEIVED:
			printk("App: Received an RCPE = %02X\n",MHL_TX_EVENT_RCPE_RECEIVED);
			break;

		default:
			break;
	}
}


#if (VBUS_POWER_CHK == ENABLE)
///////////////////////////////////////////////////////////////////////////////
//
// AppVbusControl
//
// This function or macro is invoked from MhlTx driver to ask application to
// control the VBUS power. If powerOn is sent as non-zero, one should assume
// peer does not need power so quickly remove VBUS power.
//
// if value of "powerOn" is 0, then application must turn the VBUS power on
// within 50ms of this call to meet MHL specs timing.
//
// Application module must provide this function.
///////////////////////////////////////////
void	AppVbusControl( bool_t powerOn )
{
	if( powerOn )
	{
		MHLSinkOrDonglePowerStatusCheck();
		printk("App: Peer's POW bit is set. Turn the VBUS power OFF here.\n");
	}
	else
	{
		printk("App: Peer's POW bit is cleared. Turn the VBUS power ON here.\n");
	}
}
#endif

#ifdef SiI8334DRIVER_INTERRUPT_MODE

struct work_struct	*sii8334work;
static spinlock_t sii8334_lock = SPIN_LOCK_UNLOCKED;
extern uint8_t	fwPowerState;

static void work_queue(struct work_struct *work)
{	
	uint8_t Int_count=0;
	uint8_t 				event;
	uint8_t 				eventParam;
	
	//for(Int_count=0;Int_count<15;Int_count++){
		printk(KERN_INFO "%s:%d:Int_count=%d::::::::Sii8334 interrupt happened\n", __func__,__LINE__,Int_count);
		
		SiiMhlTxGetEvents(&event, &eventParam);
	
		if( MHL_TX_EVENT_NONE != event )
			{
				AppRcpDemo( event, eventParam);
			}
			//msleep(10);	
			//if(POWER_STATE_D3 == fwPowerState)
		//		break;
//	}
	enable_irq(sii8334_PAGE_TPI->irq);
}

static irqreturn_t Sii8334_mhl_interrupt(int irq, void *dev_id)
{
	unsigned long lock_flags = 0;	 
	disable_irq_nosync(irq);
	spin_lock_irqsave(&sii8334_lock, lock_flags);	
	//printk("The sii8334 interrupt handeler is working..\n");  
	printk("The most of sii8334 interrupt work will be done by following tasklet..\n");  

	schedule_work(sii8334work);

	//printk("The sii8334 interrupt's top_half has been done and bottom_half will be processed..\n");  
	spin_unlock_irqrestore(&sii8334_lock, lock_flags);
	return IRQ_HANDLED;
}
#else
/*****************************************************************************/
/**
 *  @brief Thread function that periodically polls for MHLTx events.
 *
 *  @param[in]	data	Pointer to driver context structure
 *
 *  @return		Always returns zero when the thread exits.
 *
 *****************************************************************************/
static int EventThread(void *data)
{
	uint8_t 				event;
	uint8_t 				eventParam;


	printk("%s EventThread starting up\n", MHL_DRIVER_NAME);

	while (true)
	{
		if (kthread_should_stop())
		{
			printk("%s EventThread exiting\n", MHL_DRIVER_NAME);
			break;
		}

		HalTimerWait(EVENT_POLL_INTERVAL_30_MS);
		SiiMhlTxGetEvents(&event, &eventParam);
	
		if( MHL_TX_EVENT_NONE != event )
			{
				AppRcpDemo( event, eventParam);
			}
	}
	return 0;
}


/***** public functions ******************************************************/


/*****************************************************************************/
/**
 * @brief Start drivers event monitoring thread.
 *
 *****************************************************************************/
void StartEventThread(void)
{
	gDriverContext.pTaskStruct = kthread_run(EventThread,
											 &gDriverContext,
											 MHL_DRIVER_NAME);
}



/*****************************************************************************/
/**
 * @brief Stop driver's event monitoring thread.
 *
 *****************************************************************************/
void  StopEventThread(void)
{
	kthread_stop(gDriverContext.pTaskStruct);

}
#endif

static struct i2c_device_id mhl_Sii8334_idtable[] = {
	{"sii8334_PAGE_TPI", 0},
	{"sii8334_PAGE_TX_L0", 0},
	{"sii8334_PAGE_TX_L1", 0},
	{"sii8334_PAGE_TX_2", 0},
	{"sii8334_PAGE_TX_3", 0},
	{"sii8334_PAGE_CBUS", 0},
};

/*
 * i2c client ftn.
 */
static int __devinit mhl_Sii8334_probe(struct i2c_client *client,
			const struct i2c_device_id *dev_id)
{
	int ret = 0;

	if(match_id(&mhl_Sii8334_idtable[0], client))
	{
		sii8334_PAGE_TPI = client;
		printk("[MHL] attached %s "
			"into i2c adapter successfully\n", dev_id->name);
	}
	/*
	else if(match_id(&mhl_Sii8334_idtable[1], client))
	{
		sii8334_PAGE_TX_L0 = client;
		printk("[MHL] attached %s "
			"into i2c adapter successfully \n", dev_id->name);
	}
	*/
	else if(match_id(&mhl_Sii8334_idtable[2], client))
	{
		sii8334_PAGE_TX_L1 = client;
		printk("[MHL] attached %s "
			"into i2c adapter successfully \n", dev_id->name);
	}
	else if(match_id(&mhl_Sii8334_idtable[3], client))
	{
		sii8334_PAGE_TX_2 = client;
		printk("[MHL] attached %s "
			"into i2c adapter successfully\n", dev_id->name);
	}
	else if(match_id(&mhl_Sii8334_idtable[4], client))
	{
		sii8334_PAGE_TX_3 = client;
		printk("[MHL] attached %s "
			"into i2c adapter successfully\n", dev_id->name);
	}
	else if(match_id(&mhl_Sii8334_idtable[5], client))
	{
		sii8334_PAGE_CBUS = client;
		printk("[MHL] attached %s "
			"into i2c adapter successfully\n", dev_id->name);
	}
	else
	{
		printk("[MHL] invalid i2c adapter: can not found dev_id matched\n");
		return -EIO;
	}


	if(sii8334_PAGE_TPI != NULL 
		//&&sii8334_PAGE_TX_L0 != NULL 
		&&sii8334_PAGE_TX_L1 != NULL 
		&& sii8334_PAGE_TX_2 != NULL
		&& sii8334_PAGE_TX_3 != NULL
		&& sii8334_PAGE_CBUS != NULL)
	{
		// Announce on RS232c port.
		//
		printk("[MHL] SiI-8334 Driver Version based on 8051 driver Version 1.14 \n");
		
		if(false == Sii8334_mhl_reset()){
			printk("[MHL] Can't find the reset function in your platform file============================================\n");
			return -EIO;
			}

		//
		// Initialize the registers as required. Setup firmware vars.
		//
	
		SiiMhlTxInitialize();
		
		#ifdef SiI8334DRIVER_INTERRUPT_MODE
		sii8334work = kmalloc(sizeof(*sii8334work), GFP_ATOMIC);
		INIT_WORK(sii8334work, work_queue); 
		
		ret = request_irq(sii8334_PAGE_TPI->irq, Sii8334_mhl_interrupt, IRQ_TYPE_LEVEL_LOW,
					  sii8334_PAGE_TPI->name, sii8334_PAGE_TPI);
		if (ret)
			printk(KERN_INFO "[MHL] %s:%d:Sii8334 interrupt failed\n", __func__,__LINE__);	
			//free_irq(irq, iface);
		else{
			enable_irq_wake(sii8334_PAGE_TPI->irq);	
			//printk(KERN_INFO "%s:%d:Sii8334 interrupt successed\n", __func__,__LINE__);	
			}
		#else
		StartEventThread();		/* begin monitoring for events if using polling mode*/
		#endif
	}
	return ret;
}

static int mhl_Sii8334_remove(struct i2c_client *client)
{
	printk("[MHL] detached s5p_mhl "
		"from i2c adapter successfully\n");

	return 0;
}

static int mhl_Sii8334_suspend(struct i2c_client *cl, pm_message_t mesg)
{
	return 0;
};

static int mhl_Sii8334_resume(struct i2c_client *cl)
{
	return 0;
};


MODULE_DEVICE_TABLE(i2c, mhl_Sii8334_idtable);

static struct i2c_driver mhl_Sii8334_driver = {
	.driver = {
		.name = "Sii8334_Driver",
	},
	.id_table 	= mhl_Sii8334_idtable,
	.probe 		= mhl_Sii8334_probe,
	.remove 	= __devexit_p(mhl_Sii8334_remove),

	.suspend	= mhl_Sii8334_suspend,
	.resume 	= mhl_Sii8334_resume,
};

static int __init mhl_Sii8334_init(void)
{
	return i2c_add_driver(&mhl_Sii8334_driver);
}

static void __exit mhl_Sii8334_exit(void)
{
	i2c_del_driver(&mhl_Sii8334_driver);
}


late_initcall(mhl_Sii8334_init);
module_exit(mhl_Sii8334_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("gary <qiang.yuan@siliconimage.com>, Silicon image SZ office, Inc.");
MODULE_DESCRIPTION("sii8334 transmitter Linux driver");
MODULE_ALIAS("platform:MHL_sii8334");

