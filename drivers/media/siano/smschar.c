/****************************************************************

Siano Mobile Silicon, Inc.
MDTV receiver kernel modules.
Copyright (C) 2006-2008, Uri Shkolnik

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

 This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************/
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk() */
#include <linux/fs.h>		/* everything... */
#include <linux/types.h>	/* size_t */
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <asm/system.h>		/* cli(), *_flags */
#include <linux/uaccess.h>	/* copy_*_user */
#include <linux/slab.h> // add by jiabf

//#include <mach/mfp-pxa9xx.h>
//#include <mach/mfp-pxa3xx.h>
#include <mach/gpio.h>
#include "smscoreapi.h"

#ifdef ZTE_CMMB_ZM2167 /* add by jiabf 20120524+++++  */
#include "zi_cmmb_api.h"

extern Zi_INT32 cmmb_chip_type; 
Zi_INT32 smschar_cmmb_flag =0;
extern Zi_UINT8 Zi_CmmbCmdData[32];
extern Zi_UINT8 Zi_CmmbFrameHeadrData[32];
extern Zi_UINT8 Zi_Cmmb7816Data[32];
extern Zi_UINT8 g_7816cmd[256];
extern Zi_UINT8 g_7816bufLen;
extern Zi_DEVICE_HANDLE* Zi_pCmmbDevice;
#endif                /* add by jiabf 20120524+++++  */

#include "smscharioctl.h"
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
//#define ZTE_WAIT_EVENT_MODIFY
#ifdef ZTE_CMMB_LOOP_TEST
/*add by wangtao for cmmb 607loop 20111114 ++*/
extern u32 g_607Buf[];
/*add by wangtao for cmmb 607loop 20111114 --*/
#endif

/* max number of packets allowed to be pending on queue*/
#define SMS_CHR_MAX_Q_LEN	15
#define SMSCHAR_NR_DEVS	        17 	

struct smschar_device_t {
	struct cdev cdev;	/*!< Char device structure */
	wait_queue_head_t waitq;	/* Processes waiting */
	int cancel_waitq;
	spinlock_t lock;	/*!< critical section */
	int pending_count;
	struct list_head pending_data;	/*!< list of pending data */
	struct smscore_buffer_t *currentcb;
	int device_index;
	struct smscore_device_t *coredev;
	struct smscore_client_t *smsclient;
};

/*!  Holds the major number of the device node. may be changed at load
time.*/
int smschar_major = 0;// 251; /*ZTE:add by wangtao for spi 20110719*/

/*!  Holds the first minor number of the device node.
may be changed at load time.*/
int smschar_minor;  /*= 0*/

/* macros that allow the load time parameters change*/
module_param(smschar_major, int, S_IRUGO);
module_param(smschar_minor, int, S_IRUGO);

struct smschar_device_t smschar_devices[SMSCHAR_NR_DEVS];
static int g_smschar_inuse =0 ;

static int g_pnp_status_changed = 1;
//wait_queue_head_t g_pnp_event;

static struct class *smschr_dev_class;
static int g_has_suspended =0 ;
// static struct device* sms_power_dev ;

//int        sms_suspend_count  ;
//static struct     semaphore sem;
static int        g_has_opened=0;
static int        g_has_opened_first=0;
static int resume_flag=0;

#ifdef ZTE_CMMB_ZM2167
extern  int  sms_wake_up_spi(int data_type);
void  smschar_interrupt(void)
{
	int  ndata_type = 0;
	smschar_cmmb_flag++;

	 if( Zi_CmmbFrameHeadrData[0]> 0 )
		ndata_type = 607;
		
	 if( sms_wake_up_spi(ndata_type) != 0 )
	 {
	 	if( Zi_CmmbCmdData[0] )
			Zi_CmmbCmdData[0]--;
		if( Zi_CmmbFrameHeadrData[0] )
			Zi_CmmbFrameHeadrData[0]--;
		if( Zi_Cmmb7816Data[0] )
			Zi_Cmmb7816Data[0]--;
		smschar_cmmb_flag--;
		Zi_log("smschar_cmmb_flag = %d, Cmd[0] = %d, frame[0] = %d, 7816[0] = %d\n",
			smschar_cmmb_flag, Zi_CmmbCmdData[0], Zi_CmmbFrameHeadrData[0], Zi_Cmmb7816Data[0]);
	}
}

Zi_VOID smschar_parseMsgl(Zi_UINT8 *buffer)
{
	Zi_UINT32 flag;
	flag = (*(buffer+1)<<8) | *buffer;
	Zi_log("smschar_parseMsgl flag = %d\n", flag);
      
	switch (flag){
	case MSG_SMS_INIT_DEVICE_REQ:
		{
			Zi_log("MSG_SMS_INIT_DEVICE_REQ\n");
			Zi_CmmbCmdData[0]++;
			Zi_CmmbCmdData[7] = Zi_FRAME_TYPE_COMMAND_RESPONSE;
			Zi_CmmbCmdData[8] = Zi_CTRL_CMD_TYPE_INIT;
			Zi_CmmbCmdData[9] = 0;
			smschar_interrupt();
			break;
		}
	case MSG_SMS_GET_VERSION_EX_REQ:
		{
			Zi_log("MSG_SMS_GET_VERSION_EX_REQ\n");
			Zi_CmmbCmdData[0]++;
			Zi_CmmbCmdData[7] = Zi_FRAME_TYPE_COMMAND_RESPONSE;
			Zi_CmmbCmdData[8] = Zi_CTRL_CMD_TYPE_GET_FIRMWARE_VERSION;
			Zi_CmmbCmdData[9] = 0;
			smschar_interrupt();
			break;
		}
	case MSG_SMS_RF_TUNE_REQ:
		{
			Zi_UINT32 freqpoint;
			freqpoint = *(buffer+8) | (*(buffer+9)<<8) | (*(buffer+10)<<16) | (*(buffer+11)<<24);
			Zi_log("MSG_SMS_RF_TUNE_REQ, Freq = %d\n", freqpoint);
			Zi_CmdSetFreq(freqpoint);
			break;
		}
	case MSG_SMS_CMMB_START_CONTROL_INFO_REQ:
		{
			Zi_log("MSG_SMS_CMMB_START_CONTROL_INFO_REQ\n");
			Zi_Cmmb_Select_Service(0);
			break;
		}
	case MSG_SMS_CMMB_STOP_CONTROL_INFO_REQ:
		{
			Zi_log("MSG_SMS_CMMB_STOP_CONTROL_INFO_REQ\n");
			Zi_Cmmb_Stop_Service(0);
			break;
		}
	case MSG_SMS_CMMB_START_SERVICE_REQ:
		{
			Zi_UINT32 serviceId;
			serviceId = *(buffer+8) | (*(buffer+9)<<8) | (*(buffer+10)<<16) | (*(buffer+11)<<24);
			Zi_log("MSG_SMS_CMMB_START_SERVICE_REQ, ServiceId = %d\n", serviceId);
			Zi_Cmmb_Select_Service(serviceId);
			break;
		}
	case MSG_SMS_CMMB_STOP_SERVICE_REQ:
		{
			Zi_UINT32 channelNum, serviceId;
			channelNum = *(buffer+8) | (*(buffer+9)<<8) | (*(buffer+10)<<16) | (*(buffer+11)<<24);
			if(Zi_pCmmbDevice)
			{
				serviceId = Zi_pCmmbDevice->ActiveService[channelNum - 2].nActiveServiceID;
			Zi_log("MSG_SMS_CMMB_START_SERVICE_REQ, ServiceId = %d\n", serviceId);
			Zi_Cmmb_Stop_Service(serviceId);
			}
			break;
		}
	case MSG_SMS_SMART_CARD_INIT_REQ:
		{
			Zi_log("MSG_SMS_SMART_CARD_INIT_REQ\n");
			Zi_CmdISO7816Reset();
			break;
		}
	case MSG_SMS_SMART_CARD_WRITE_REQ:
		{
			Zi_UINT32 CmdLength;
			Zi_UINT8 *uambuffer;
			Zi_log("MSG_SMS_SMART_CARD_WRITE_REQ\n");
			CmdLength = ((*(buffer+5)<<8)|*(buffer+4))-12;
			uambuffer = buffer+12;
			
			Zi_memcpy(g_7816cmd, uambuffer, CmdLength);
			g_7816bufLen = CmdLength;
			
			Zi_CmdAPDU(uambuffer,CmdLength);
			break;
		}
	case MSG_SMS_GET_STATISTICS_EX_REQ :
		{
			Zi_INT32 Zi_RSSI =0;
			if(Zi_pCmmbDevice)
			{
				Zi_RSSI = Zi_pCmmbDevice->Zi_SysStatus->nSignalStrength;
				Zi_GetSignalStatus(Zi_pCmmbDevice);
				if(Zi_pCmmbDevice->Zi_SysStatus->nSignalStrength < 190)
				{
					Zi_pCmmbDevice->Zi_SysStatus->nSignalStrength += 2;
				}
				if((Zi_RSSI - Zi_pCmmbDevice->Zi_SysStatus->nSignalStrength <= 8 &&
				   Zi_RSSI - Zi_pCmmbDevice->Zi_SysStatus->nSignalStrength >= 3) ||
				   (Zi_RSSI - Zi_pCmmbDevice->Zi_SysStatus->nSignalStrength <= -3 &&
				   Zi_RSSI - Zi_pCmmbDevice->Zi_SysStatus->nSignalStrength >= -8))
				{
					Zi_pCmmbDevice->Zi_SysStatus->nSignalStrength = (Zi_RSSI+ Zi_pCmmbDevice->Zi_SysStatus->nSignalStrength)/2;
				}
				
				Zi_CmmbCmdData[0]++;
				Zi_CmmbCmdData[7] = Zi_FRAME_TYPE_COMMAND_RESPONSE;
				Zi_CmmbCmdData[8] = Zi_CTRL_CMD_TYPE_GET_SIGNAL_POWER;
				Zi_CmmbCmdData[10] =  Zi_pCmmbDevice->Zi_SysStatus->nSignalStrength;
				Zi_CmmbCmdData[11] = Zi_pCmmbDevice->Zi_SysStatus->nSignalQuality;
				Zi_CmmbCmdData[12] = Zi_pCmmbDevice->Zi_SysStatus->nSignalNoiseRatio;
				Zi_CmmbCmdData[13] = Zi_pCmmbDevice->Zi_SysStatus->nLdpcErrPercent;
				Zi_CmmbCmdData[14] = Zi_pCmmbDevice->Zi_SysStatus->nCurrentFrequency & 0xff;
				Zi_CmmbCmdData[15] = (Zi_pCmmbDevice->Zi_SysStatus->nCurrentFrequency >> 8) & 0xff;
				Zi_CmmbCmdData[16] = (Zi_pCmmbDevice->Zi_SysStatus->nCurrentFrequency >> 16) & 0xff;
				Zi_CmmbCmdData[17] = (Zi_pCmmbDevice->Zi_SysStatus->nCurrentFrequency >> 24) & 0xff;
				smschar_interrupt();
			}
			break;
		}        
	default:
		return -ENOIOCTLCMD;
	}
}

int smschar_wakeup_spi(void *context)
{
	struct smschar_device_t *dev = context;
	//printk(KERN_EMERG"[zm2167] int smschar_wakeup_spi   dev=%d\n",dev);
	if( dev != NULL )
	{
	    	wake_up_interruptible(&dev->waitq);
		return 0;
	}

	return 1;
}
#endif
/**
 * unregisters sms client and returns all queued buffers
 *
 * @param dev pointer to the client context (smschar parameters block)
 *
 */
static void smschar_unregister_client(struct smschar_device_t *dev)
{
	unsigned long flags;

	sms_info("entering... smschar_unregister_client....\n");
	if (dev->coredev && dev->smsclient) {
		dev->cancel_waitq = 1;
		#ifdef ZTE_WAIT_EVENT_MODIFY
		wake_up(&dev->waitq); // modify by jiabf 20110811
		#else
            	wake_up_interruptible(&dev->waitq);
		#endif

		spin_lock_irqsave(&dev->lock, flags);

		while (!list_empty(&dev->pending_data)) {
			struct smscore_buffer_t *cb =
			    (struct smscore_buffer_t *)dev->pending_data.next;
			list_del(&cb->entry);

			smscore_putbuffer(dev->coredev, cb);
			dev->pending_count--;
		}

		if (dev->currentcb) {
			smscore_putbuffer(dev->coredev, dev->currentcb);
			dev->currentcb = NULL;
			dev->pending_count--;
		}

		smscore_unregister_client(dev->smsclient);
		dev->smsclient = NULL;

		spin_unlock_irqrestore(&dev->lock, flags);
	}
}

/**
 * queues incoming buffers into buffers queue
 *
 * @param context pointer to the client context (smschar parameters block)
 * @param cb pointer to incoming buffer descriptor
 *
 * @return 0 on success, <0 on queue overflow.
 */
static int smschar_onresponse(void *context, struct smscore_buffer_t *cb)
{
	struct smschar_device_t *dev = context;
	unsigned long flags;

	if (!dev) {
		sms_err("recieved bad dev pointer\n");
		return -EFAULT;
	}
	spin_lock_irqsave(&dev->lock, flags);

	if (dev->pending_count > SMS_CHR_MAX_Q_LEN) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EBUSY;
	}

	dev->pending_count++;
	/* if data channel, remove header */
	if (dev->device_index) {
		cb->size -= sizeof(struct SmsMsgHdr_ST);
		cb->offset += sizeof(struct SmsMsgHdr_ST);
	}

	list_add_tail(&cb->entry, &dev->pending_data);
	spin_unlock_irqrestore(&dev->lock, flags);

	if (waitqueue_active(&dev->waitq))
             #ifdef ZTE_WAIT_EVENT_MODIFY
                wake_up(&dev->waitq); // modify by jiabf 20110811
             #else
             wake_up_interruptible(&dev->waitq);
             #endif
	return 0;
}

/**
 * handles device removal event
 *
 * @param context pointer to the client context (smschar parameters block)
 *
 */
static void smschar_onremove(void *context)
{
	struct smschar_device_t *dev = (struct smschar_device_t *)context;

	smschar_unregister_client(dev);
	dev->coredev = NULL;
}

/**
 * registers client associated with the node
 *
 * @param inode Inode concerned.
 * @param file File concerned.
 *
 * @return 0 on success, <0 on error.
 */
static int smschar_open(struct inode *inode, struct file *file)
{
	struct smschar_device_t *dev = container_of(inode->i_cdev,
						    struct smschar_device_t,
						    cdev);
	int rc = -ENODEV;

       // if(g_has_suspended)
         //  return rc;

	   printk(KERN_WARNING"helike enter smschar_open\n");
	sms_info("entering index %d\n", dev->device_index);

	if (dev->coredev) {
		struct smsclient_params_t params;
      #if 1

		if(g_has_opened_first==0 && dev->device_index==0)
		{
#ifdef ZTE_CMMB_ZM2167
		smschar_cmmb_flag = 0;
#endif
		 smsspi_poweron();
		 g_has_opened_first=1;
		 printk("open first********\n");

	       }
	       else if(dev->device_index!=0)
                g_has_opened_first=0;
      /****************end*******************************/ 
      #endif           		
	//	down(&sem);
		params.initial_id = dev->device_index ?
		    dev->device_index : SMS_HOST_LIB;
		params.data_type = dev->device_index ? MSG_SMS_DAB_CHANNEL : 0;
		params.onresponse_handler = smschar_onresponse;
		params.onremove_handler = smschar_onremove;
		params.context = dev;

		rc = smscore_register_client(dev->coredev, &params,
					     &dev->smsclient);
		if (!rc)
			file->private_data = dev;
		dev->cancel_waitq = 0;
		g_pnp_status_changed = 1;
	        g_has_opened++;	
	//	up(&sem);
	}

       
	if (rc)
		sms_err(" exiting, rc %d\n", rc);
		   printk(KERN_WARNING"helike exit smschar_open rc=%d\n",rc);

	return rc;
}

/**
 * unregisters client associated with the node
 *
 * @param inode Inode concerned.
 * @param file File concerned.
 *
 */
static int smschar_release(struct inode *inode, struct file *file)
{
	struct smschar_device_t *dev = file->private_data;
/*        if(g_has_suspended ){
            printk(KERN_EMERG "SMS1180: suspenede has released all client\n");
            return 0;
        }
*/

    //printk("release smschar,%d\n",g_has_opened);

	smschar_unregister_client(file->private_data);
    #if 1

	if(!(--g_has_opened)&& (g_has_opened_first==0))
	{
#ifdef ZTE_CMMB_ZM2167
		if(CMMB_ZM2167_DEVICE == cmmb_chip_type)
			Zi_CloseDevice(Zi_pCmmbDevice);	//add by mingsen
#endif
	     smscore_reset_device_drvs(dev->coredev);
             smsspi_off();
             g_has_opened_first=0;
	     printk("release at the end******\n");
        }
      /*****************end**************************/
    #endif
	sms_info("exiting\n");
	return 0;
}

/**
 * copies data from buffers in incoming queue into a user buffer
 *
 * @param file File structure.
 * @param buf Source buffer.
 * @param count Size of source buffer.
 * @param f_pos Position in file (ignored).
 *
 * @return Number of bytes read, or <0 on error.
 */
static ssize_t smschar_read(struct file *file, char __user *buf,
			    size_t count, loff_t *f_pos)
{
	struct smschar_device_t *dev = file->private_data;
	unsigned long flags;
	int rc, copied = 0;

	if (!buf) {
		sms_err("Bad pointer recieved from user.\n");
		return -EFAULT;
	}
	if (!dev->coredev || !dev->smsclient||g_has_suspended) {
		sms_err("no client\n");
		return -ENODEV;
	}
	rc = wait_event_interruptible(dev->waitq,
				      !list_empty(&dev->pending_data)
				      || (dev->cancel_waitq));
	if (rc < 0) {
		sms_err("wait_event_interruptible error %d\n", rc);
		return rc;
	}
	if (dev->cancel_waitq)
		return 0;
	if (!dev->smsclient) {
		sms_err("no client\n");
		return -ENODEV;
	}
	spin_lock_irqsave(&dev->lock, flags);

	while (!list_empty(&dev->pending_data) && (copied < count)) {
		struct smscore_buffer_t *cb =
		    (struct smscore_buffer_t *)dev->pending_data.next;
		int actual_size = min(((int)count - copied), cb->size);
		if (copy_to_user(&buf[copied], &((char *)cb->p)[cb->offset],
				 actual_size)) {
			sms_err("copy_to_user failed\n");
			spin_unlock_irqrestore(&dev->lock, flags);
			return -EFAULT;
		}
		copied += actual_size;
		cb->offset += actual_size;
		cb->size -= actual_size;

		if (!cb->size) {
			list_del(&cb->entry);
			smscore_putbuffer(dev->coredev, cb);
			dev->pending_count--;
		}
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return copied;
}

/**
 * sends the buffer to the associated device
 *
 * @param file File structure.
 * @param buf Source buffer.
 * @param count Size of source buffer.
 * @param f_pos Position in file (ignored).
 *
 * @return Number of bytes read, or <0 on error.
 */
static ssize_t smschar_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *f_pos)
{
	struct smschar_device_t *dev;
	void *buffer;

         
	if (file == NULL) {
		sms_err("file is NULL\n");
		return EINVAL;
	}

	if (file->private_data == NULL) {
		sms_err("file->private_data is NULL\n");
		return -EINVAL;
	}

	dev = file->private_data;
	if (!dev->smsclient||g_has_suspended) {
		sms_err("no client\n");
		return -ENODEV;
	}

	buffer = kmalloc(ALIGN(count, SMS_ALLOC_ALIGNMENT) + SMS_DMA_ALIGNMENT,
			 GFP_KERNEL | GFP_DMA);
	if (buffer) {
		void *msg_buffer = (void *)SMS_ALIGN_ADDRESS(buffer);

		if (!copy_from_user(msg_buffer, buf, count))
			smsclient_sendrequest(dev->smsclient,
					      msg_buffer, count);
		else
			count = 0;

		kfree(buffer);
	}

	return count;
}

static int smschar_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct smschar_device_t *dev = file->private_data;
	return smscore_map_common_buffer(dev->coredev, vma);
}

/**
 * waits until buffer inserted into a queue. when inserted buffer offset
 * are reportedto the calling process. previously reported buffer is
 * returned to smscore pool.
 *
 * @param dev pointer to smschar parameters block
 * @param touser pointer to a structure that receives incoming buffer offsets
 *
 * @return 0 on success, <0 on error.
 */
static int smschar_wait_get_buffer(struct smschar_device_t *dev,
				   struct smschar_buffer_t *touser)
{
	unsigned long flags;
	int rc;
#ifdef ZTE_CMMB_ZM2167
	if( CMMB_ZM2167_DEVICE == cmmb_chip_type )
	{
		rc = wait_event_interruptible(dev->waitq, (smschar_cmmb_flag > 0) || (dev->cancel_waitq) );  
  
		Zi_log("dev=%d smschar_cmmb_flag=%d\n", dev, smschar_cmmb_flag);
		if (rc < 0)
		{
			Zi_log("error, rc=%d\n", rc);
			return rc;
		}

		smschar_cmmb_flag > 0 ? smschar_cmmb_flag-- : (smschar_cmmb_flag = 0);
	}
	else
#endif
	{
	spin_lock_irqsave(&dev->lock, flags);

	if (dev->currentcb) {
		smscore_putbuffer(dev->coredev, dev->currentcb);
		dev->currentcb = NULL;
		dev->pending_count--;
	}

	spin_unlock_irqrestore(&dev->lock, flags);


	memset(touser, 0, sizeof(struct smschar_buffer_t));
    
        #ifdef ZTE_WAIT_EVENT_MODIFY   //modify by jiabf 20110811
        wait_event(dev->waitq,
			!list_empty(&dev->pending_data)
			|| (dev->cancel_waitq));
        #else
	rc = wait_event_interruptible(dev->waitq,
				      !list_empty(&dev->pending_data)
				      || (dev->cancel_waitq));
	if (rc < 0) {
		sms_err("wait_event_interruptible error, rc=%d\n", rc);
		return rc;
	}
        #endif
	if (dev->cancel_waitq) {
		touser->offset = 0;
		touser->size = 0;
		return 0;
	}
	if (!dev->smsclient) {
		sms_err("no client\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&dev->lock, flags);

	if (!list_empty(&dev->pending_data)) {
		struct smscore_buffer_t *cb =
		    (struct smscore_buffer_t *)dev->pending_data.next;
		touser->offset = cb->offset_in_common + cb->offset;
		touser->size = cb->size;

		list_del(&cb->entry);

		dev->currentcb = cb;
	} else {
		touser->offset = 0;
		touser->size = 0;
	}

	spin_unlock_irqrestore(&dev->lock, flags);
	}
	return 0;
}

/**
 * poll for data availability
 *
 * @param file File structure.
 * @param wait kernel polling table.
 *
 * @return POLLIN flag if read data is available.
 */
static unsigned int smschar_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct smschar_device_t *dev;
	int mask = 0;

	if (file == NULL) {
		sms_err("file is NULL\n");
		return EINVAL;
	}

	if (file->private_data == NULL) {
		sms_err("file->private_data is NULL\n");
		return -EINVAL;
	}

	dev = file->private_data;

	if (list_empty(&dev->pending_data)) {
		sms_info("No data is ready, waiting for data recieve.\n");
		poll_wait(file, &dev->waitq, wait);
	}

	if (!list_empty(&dev->pending_data))
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

static long smschar_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct smschar_device_t *dev = file->private_data;
	void __user *up = (void __user *)arg;

	if (!dev->coredev || !dev->smsclient||g_has_suspended) {
		sms_err("no client\n");
		return -ENODEV;
	}
    
//	sms_info("smscharioctl - command is 0x%x", cmd);
	switch (cmd) {
	case SMSCHAR_STARTUP:
                smsspi_poweron();
		return 0;
	case SMSCHAR_SET_DEVICE_MODE:
#ifdef ZTE_CMMB_ZM2167
		if(CMMB_ZM2167_DEVICE == cmmb_chip_type)
			return 0;
		else
#endif
		return smscore_set_device_mode(dev->coredev, (int)arg);

	case SMSCHAR_GET_DEVICE_MODE:
		{
			if (put_user(smscore_get_device_mode(dev->coredev),
				     (int *)up))
				return -EFAULT;
			break;
		}
	case SMSCHAR_IS_DEVICE_PNP_EVENT:
		{
                       printk("pnp event not supported\n") ;
#if 0
			sms_info("Waiting for PnP event.\n");
			wait_event_interruptible(g_pnp_event,
						 !g_pnp_status_changed);
			g_pnp_status_changed = 0;
			sms_info("PnP Event %d.\n", g_smschar_inuse);
			if (put_user(g_smschar_inuse, (int *)up))
				return -EFAULT;
#endif 
			break;
		}
	case SMSCHAR_GET_BUFFER_SIZE:
		{

			if (put_user
			    (smscore_get_common_buffer_size(dev->coredev),
			     (int *)up))
				return -EFAULT;

			break;
		}

	case SMSCHAR_WAIT_GET_BUFFER:
		{
			struct smschar_buffer_t touser;
			int rc;

			rc = smschar_wait_get_buffer(dev, &touser);
			if (rc < 0)
				return rc;
#ifdef ZTE_CMMB_ZM2167
			if( CMMB_ZM2167_DEVICE == cmmb_chip_type)
			{
				//printk("[zm2167] smschar_wait_get_buffer cmd[0] = 0x%x frame[0] = 0x%x 7816[0] = 0x%x\n", Zi_CmmbCmdData[0],Zi_CmmbFrameHeadrData[0],Zi_Cmmb7816Data[0]);
				if(Zi_CmmbCmdData[0]==0 && Zi_CmmbFrameHeadrData[0]==0 && Zi_Cmmb7816Data[0]==0)
				{
					//mdelay(10);
					return -1;
				}
				if( Zi_CmmbCmdData[0] )
				{
					copy_to_user(up, Zi_CmmbCmdData,32);
					Zi_CmmbCmdData[0]--;
				  	return 0;
				}
				if( Zi_Cmmb7816Data[0] )
				{
					copy_to_user(up, Zi_Cmmb7816Data,32);
					Zi_Cmmb7816Data[0]--;
				  	return 0;
				}
				if( Zi_CmmbFrameHeadrData[0] )
				{
					copy_to_user(up, Zi_CmmbFrameHeadrData,32);
					Zi_CmmbFrameHeadrData[0]--;
					return 0;
				}
			}
			else
#endif
			{
			if (copy_to_user(up, &touser,
					 sizeof(struct smschar_buffer_t)))
				return -EFAULT;
			}
			break;
		}
	case SMSCHAR_CANCEL_WAIT_BUFFER:
		{
			dev->cancel_waitq = 1;
                        #ifdef ZTE_WAIT_EVENT_MODIFY
                        wake_up(&dev->waitq); //modify by jiabf 20110811
                        #else
                        wake_up_interruptible(&dev->waitq);
                        #endif
			break;
		}
	case SMSCHAR_GET_FW_FILE_NAME:
		{
			if (!up)
				return -EINVAL;
#ifdef ZTE_CMMB_ZM2167
			if( CMMB_ZM2167_DEVICE == cmmb_chip_type )
				return 0;
			else
#endif
			return smscore_get_fw_filename(dev->coredev,
				       ((struct
					 smschar_get_fw_filename_ioctl_t
					 *)up)->mode,
				       ((struct
					 smschar_get_fw_filename_ioctl_t
					 *)up)->filename);
		}
	case SMSCHAR_SEND_FW_FILE:
		{
			if (!up)
				return -EINVAL;
#ifdef ZTE_CMMB_ZM2167
			if( CMMB_ZM2167_DEVICE == cmmb_chip_type )
				return 0;
			else
#endif
			return smscore_send_fw_file(dev->coredev,
					((struct
					smschar_send_fw_file_ioctl_t
					*)up)->fw_buf,
					((struct
					smschar_send_fw_file_ioctl_t
					*)up)->fw_size);
		}
	// leadcore add on 2010-01-07
	case  SMSCHAR_GET_RESUME_FLAG:
		 if (copy_to_user(up, &resume_flag, sizeof(int)))
			  return -EFAULT;
		 else
		 return 0;
    		  
	case  SMSCHAR_SET_RESUME_FLAG:
		 if (copy_from_user(&resume_flag,up, sizeof(int)))
		 	return -EFAULT;
		 else
	     return 0;

		  
	case  SMSCHAR_RESET_DEVICE_DRVS:
              smsspi_off();
	     return  smscore_reset_device_drvs (dev->coredev);
         
	#ifdef ZTE_CMMB_LOOP_TEST
	/*add by wangtao for cmmb 607loop 20111114 ++*/		 
	case  SMSCHAR_607_CALLBACK:	
		 if (copy_to_user(up, g_607Buf,65535*4))
			  return -EFAULT;
		 else 
		 return 0;
	/*add by wangtao for cmmb 607loop 20111114 --*/	
       #endif
#ifdef ZTE_CMMB_ZM2167
	case SMSCHAR_GET_CHIP_TYPE:
		Zi_log("SMSCHAR_GET_CHIP_TYPE");
		put_user(cmmb_chip_type, (int *)up);
		break;
#endif    
	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}


struct file_operations smschar_fops = {
	.owner = THIS_MODULE,
	.read = smschar_read,
	.write = smschar_write,
	.open = smschar_open,
	.release = smschar_release,
	.mmap = smschar_mmap,
	.poll = smschar_poll,
	.unlocked_ioctl = smschar_ioctl,                // modify by jiabf
};

static int smschar_setup_cdev(struct smschar_device_t *dev, int index)
{
	//struct device *smschr_dev;
	int rc, devno = MKDEV(smschar_major, smschar_minor + index);

	cdev_init(&dev->cdev, &smschar_fops);

	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &smschar_fops;

	kobject_set_name(&dev->cdev.kobj, "Siano_sms%d", index);
	rc = cdev_add(&dev->cdev, devno, 1);
	
	if (!index)
	{
		device_create(smschr_dev_class, NULL, devno, NULL, "mdtvctrl");
	}
	else
	{	
		device_create(smschr_dev_class, NULL, devno, NULL, "mdtv%d", index);
	}
	
	sms_info("exiting %p %d, rc %d", dev, index, rc);

	return rc;
}

/**
 * smschar callback that called when device plugged in/out. the function
 * register or unregisters char device interface according to plug in/out
 *
 * @param coredev pointer to device that is being plugged in/out
 * @param device pointer to system device object
 * @param arrival 1 on plug-on, 0 othewise
 *
 * @return 0 on success, <0 on error.
 */
static int smschar_hotplug(struct smscore_device_t *coredev,
			   struct device *device, int arrival)
{
	int rc = 0, i;

	sms_info("entering %d\n", arrival);

	g_pnp_status_changed = 1;
	if (arrival) {
		/* currently only 1 instance supported */
		if (!g_smschar_inuse) {
			/* data notification callbacks assignment */
			memset(smschar_devices, 0, SMSCHAR_NR_DEVS *
			       sizeof(struct smschar_device_t));

			/* Initialize each device. */
			for (i = 0; i < SMSCHAR_NR_DEVS; i++) {
				sms_info("create device %d", i);
				smschar_setup_cdev(&smschar_devices[i], i);
				INIT_LIST_HEAD(&smschar_devices[i].
					       pending_data);
				spin_lock_init(&smschar_devices[i].lock);
				init_waitqueue_head(&smschar_devices[i].waitq);

				smschar_devices[i].coredev = coredev;
				smschar_devices[i].device_index = i;
			}
			g_smschar_inuse = 1;
//			wake_up_interruptible(&g_pnp_event);
		}
	} else {
		/* currently only 1 instance supported */
		if (g_smschar_inuse) {
			/* Get rid of our char dev entries */
			for (i = 0; i < SMSCHAR_NR_DEVS; i++) {
				cdev_del(&smschar_devices[i].cdev);
				sms_info("remove device %d\n", i);
			}

			g_smschar_inuse = 0;
//			wake_up_interruptible(&g_pnp_event);
		}
	}

	sms_info("exiting, rc %d\n", rc);

	return rc;		/* succeed */
}

void smschar_reset_device(void)
{
    int i;
    printk(KERN_EMERG "SMS1180:in smschar_reset_device\n") ;
    for(i=0;i< SMSCHAR_NR_DEVS;i++)
    {
        smschar_devices[i].cancel_waitq = 1;
        #ifdef ZTE_WAIT_EVENT_MODIFY
        wake_up(&smschar_devices[i].waitq) ; //modify by jiabf 20110811
        #else
        wake_up_interruptible(&smschar_devices[i].waitq) ;
        #endif
        smschar_unregister_client(&smschar_devices[i]) ;
    }
}
void smschar_set_suspend(int suspend_on)// 1: suspended ,0:resume  
{
    printk(KERN_EMERG "SMS1180 : suspend_on = %d\n",suspend_on) ;
    if(suspend_on) 
       g_has_suspended = 1;
    else 
       g_has_suspended = 0;
}

EXPORT_SYMBOL(smschar_reset_device) ;
EXPORT_SYMBOL(smschar_set_suspend) ;

#if 0
static ssize_t
sms_suspend_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"%d",sms_suspend_count) ;
}
static ssize_t
sms_suspend_state_store(struct device *dev, struct device_attribute *attr,
		const char *buffer, size_t count) 
{
    sms_suspend_count =0 ;
    return count ;
}
static DEVICE_ATTR(suspend,S_IRUGO|S_IWUGO,sms_suspend_state_show,sms_suspend_state_store);
#endif


#ifdef CONFIG_PM
#ifdef CONFIG_ANDROID_POWER
void smsspi_android_suspend_handler(android_early_suspend_t *h)
{
}

void smsspi_android_resume_handler(android_early_suspend_t *h)
{
	  int value;
	  if(g_has_opened)
	  {
	  resume_flag=1;
	  }
	  else
	  resume_flag=0;
}
static android_early_suspend_t smsspi_android_suspend = {
	.level = 5,
	.suspend = smsspi_android_suspend_handler,
	.resume = smsspi_android_resume_handler,
};
#endif
#endif /* CONFIG_PM */
int smschar_register(void)
{
	dev_t devno = MKDEV(smschar_major, smschar_minor);
	int rc;

	sms_info("registering device major=%d minor=%d\n", smschar_major,
		 smschar_minor);
	if (smschar_major) {
		rc = register_chrdev_region(devno, SMSCHAR_NR_DEVS, "smschar");
	} else {
		rc = alloc_chrdev_region(&devno, smschar_minor,
					 SMSCHAR_NR_DEVS, "smschar");
		smschar_major = MAJOR(devno);
	}

	if (rc < 0) {
		sms_warn("smschar: can't get major %d\n", smschar_major);
		return rc;
	}
      sms_info("registering device 1 major=%d minor=%d\n", smschar_major,
		 smschar_minor);
//	init_waitqueue_head(&g_pnp_event);

	smschr_dev_class = class_create(THIS_MODULE, "smsmdtv");
	if(IS_ERR(smschr_dev_class)){
		sms_err("Could not create sms char device class\n");
		return -1;
	}
        /*sms_power_dev = device_create(smschr_dev_class,NULL,0,"%s","power_state") ;
        if(sms_power_dev)
        {
           rc = device_create_file(sms_power_dev, &dev_attr_suspend) ;
        }*/
	//android_register_early_suspend(&smsspi_android_suspend);
	return smscore_register_hotplug(smschar_hotplug);
}

void smschar_unregister(void)
{
	dev_t devno = MKDEV(smschar_major, smschar_minor);
	
	int i;
	for( i = 0; i < SMSCHAR_NR_DEVS; i++)
		device_destroy(smschr_dev_class, MKDEV(smschar_major, i));
	class_destroy(smschr_dev_class);

	unregister_chrdev_region(devno, SMSCHAR_NR_DEVS);
	smscore_unregister_hotplug(smschar_hotplug);
	//android_unregister_early_suspend(&smsspi_android_suspend);
	sms_info("unregistered\n");
}
