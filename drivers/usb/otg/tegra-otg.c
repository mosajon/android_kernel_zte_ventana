/*
 * drivers/usb/otg/tegra-otg.c
 *
 * OTG transceiver driver for Tegra UTMI phy
 *
 * Copyright (C) 2010 NVIDIA Corp.
 * Copyright (C) 2010 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/platform_device.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/wakelock.h>
/* ++ ZTE: cuijian@20111101 */	
#include <linux/workqueue.h>
#include <linux/spinlock.h>
/* -- ZTE: cuijian@20111101 */	

#define USB_PHY_WAKEUP		0x408
#define  USB_ID_INT_EN		(1 << 0)
#define  USB_ID_INT_STATUS	(1 << 1)
#define  USB_ID_STATUS		(1 << 2)
#define  USB_ID_PIN_WAKEUP_EN	(1 << 6)
#define  USB_VBUS_WAKEUP_EN	(1 << 30)
#define  USB_VBUS_INT_EN	(1 << 8)
#define  USB_VBUS_INT_STATUS	(1 << 9)
#define  USB_VBUS_STATUS	(1 << 10)
#define  USB_INTS		(USB_VBUS_INT_STATUS | USB_ID_INT_STATUS)

/* ++ ZTE: cuijian@20111101 */	
struct tegra_otg_irqnode{
	unsigned long cur_int_status;
	struct list_head list;
};
/* -- ZTE: cuijian@20111101 */	

struct tegra_otg_data {
	struct otg_transceiver otg;
	unsigned long int_status;
	spinlock_t lock;
	void __iomem *regs;
	struct clk *clk;
	int irq;
	struct platform_device *pdev;
	struct work_struct work;
	unsigned int intr_reg_data;
	bool detect_vbus;
	bool clk_enabled;
    
/* ++ ZTE: lixingyuan@20110811 */
 	struct delayed_work charger_work;
 	spinlock_t charger_detect_lock;
	struct wake_lock vbus_lock;
/* -- ZTE: lixingyuan@20110811 */

/* ++ ZTE: cuijian@20111101 */	
	struct workqueue_struct *wq;
	struct list_head irq_queue;
/* -- ZTE: cuijian@20111101 */	
};

static struct tegra_otg_data *tegra_clone;

static inline unsigned long otg_readl(struct tegra_otg_data *tegra,
				      unsigned int offset)
{
	return readl(tegra->regs + offset);
}

static inline void otg_writel(struct tegra_otg_data *tegra, unsigned long val,
			      unsigned int offset)
{
	writel(val, tegra->regs + offset);
}

static void tegra_otg_enable_clk(void)
{
	if (!tegra_clone)
		return;
	if (!tegra_clone->clk_enabled)
		clk_enable(tegra_clone->clk);
	tegra_clone->clk_enabled = true;
}

static void tegra_otg_disable_clk(void)
{
	if (!tegra_clone)
		return;
	if (tegra_clone->clk_enabled)
		clk_disable(tegra_clone->clk);
	tegra_clone->clk_enabled = false;
}

static const char *tegra_state_name(enum usb_otg_state state)
{
	if (state == OTG_STATE_A_HOST)
		return "HOST";
	if (state == OTG_STATE_B_PERIPHERAL)
		return "PERIPHERAL";
	if (state == OTG_STATE_A_SUSPEND)
		return "SUSPEND";
	return "INVALID";
}

static struct platform_device *
tegra_usb_otg_host_register(struct platform_device *ehci_device,
			    struct tegra_ehci_platform_data *pdata)
{
	struct platform_device *pdev;
	void *platform_data;
	int val;

	pdev = platform_device_alloc(ehci_device->name, ehci_device->id);
	if (!pdev)
		return NULL;

	val = platform_device_add_resources(pdev, ehci_device->resource,
					    ehci_device->num_resources);
	if (val)
		goto error;

	pdev->dev.dma_mask =  ehci_device->dev.dma_mask;
	pdev->dev.coherent_dma_mask = ehci_device->dev.coherent_dma_mask;

	platform_data = kmalloc(sizeof(struct tegra_ehci_platform_data),
		GFP_KERNEL);
	if (!platform_data)
		goto error;

	memcpy(platform_data, pdata, sizeof(struct tegra_ehci_platform_data));
	pdev->dev.platform_data = platform_data;

	val = platform_device_add(pdev);
	if (val)
		goto error_add;

	return pdev;

error_add:
	kfree(platform_data);
error:
	pr_err("%s: failed to add the host controller device\n", __func__);
	platform_device_put(pdev);
	return NULL;
}

static void tegra_usb_otg_host_unregister(struct platform_device *pdev)
{
	kfree(pdev->dev.platform_data);
	pdev->dev.platform_data = NULL;
	platform_device_unregister(pdev);
}

void tegra_start_host(struct tegra_otg_data *tegra)
{
	struct tegra_otg_platform_data *pdata = tegra->otg.dev->platform_data;
	if (!tegra->pdev) {
		tegra->pdev = tegra_usb_otg_host_register(pdata->ehci_device,
							  pdata->ehci_pdata);
	}
}

void tegra_stop_host(struct tegra_otg_data *tegra)
{
	if (tegra->pdev) {
		tegra_usb_otg_host_unregister(tegra->pdev);
		tegra->pdev = NULL;
	}
}

static void irq_work(struct work_struct *work)
{
	struct tegra_otg_data *tegra =
		container_of(work, struct tegra_otg_data, work);
	struct otg_transceiver *otg = &tegra->otg;
	enum usb_otg_state from = otg->state;
	enum usb_otg_state to = OTG_STATE_UNDEFINED;
	unsigned long flags;
	unsigned long status;
/* ++ ZTE: cuijian@20111102 */	
	struct tegra_otg_irqnode *otg_node;
/* -- ZTE: cuijian@20111102 */		

	if (tegra->detect_vbus) {
		tegra->detect_vbus = false;
		tegra_otg_enable_clk();
		return;
	}

	clk_enable(tegra->clk);

	spin_lock_irqsave(&tegra->lock, flags);
/* ++ ZTE: cuijian@20111102 */	
	if (list_empty(&tegra->irq_queue)){
		status = tegra->int_status;
	}else{
		otg_node = list_entry( tegra->irq_queue.next, struct tegra_otg_irqnode, list);
		list_del(&otg_node->list);
		status = otg_node->cur_int_status;
		kfree(otg_node);
	}

	//status = tegra->int_status;

	if (status & USB_ID_INT_STATUS) {
/* -- ZTE: cuijian@20111102 */	
		if (status & USB_ID_STATUS) {
			if ((status & USB_VBUS_STATUS) && (from != OTG_STATE_A_HOST))
				to = OTG_STATE_B_PERIPHERAL;
			else
				to = OTG_STATE_A_SUSPEND;
		}
		else
			to = OTG_STATE_A_HOST;
	}

/* ++ ZTE: cuijian@20111102 */		
	if (from != OTG_STATE_A_HOST && to != OTG_STATE_A_HOST) {
		if (status & USB_VBUS_INT_STATUS) {
/* -- ZTE: cuijian@20111102 */	
			if (status & USB_VBUS_STATUS)
				to = OTG_STATE_B_PERIPHERAL;
			else
				to = OTG_STATE_A_SUSPEND;
		}
	}
	spin_unlock_irqrestore(&tegra->lock, flags);

	if (to != OTG_STATE_UNDEFINED) {
		otg->state = to;

		dev_info(tegra->otg.dev, "%s --> %s\n", tegra_state_name(from),
					      tegra_state_name(to));

		if (to == OTG_STATE_A_SUSPEND) {
			if (from == OTG_STATE_A_HOST)
				tegra_stop_host(tegra);
			else if (from == OTG_STATE_B_PERIPHERAL && otg->gadget)
				usb_gadget_vbus_disconnect(otg->gadget);
		} else if (to == OTG_STATE_B_PERIPHERAL && otg->gadget) {
			if (from == OTG_STATE_A_SUSPEND)
				usb_gadget_vbus_connect(otg->gadget);
		} else if (to == OTG_STATE_A_HOST) {
			if (from == OTG_STATE_A_SUSPEND)
			    tegra_start_host(tegra);
/* ++ ZTE: cuijian@20111214 */
                        //deal with vbus int before id int in host mode
			if (from == OTG_STATE_B_PERIPHERAL){
    			    usb_gadget_vbus_disconnect(otg->gadget);
    			    tegra_start_host(tegra);
    			}
/* -- ZTE: cuijian@20111214 */
		}
	}
	clk_disable(tegra->clk);
	tegra_otg_disable_clk();

/* ++ ZTE: cuijian@20111101 */
	if (!list_empty(&tegra->irq_queue))
		queue_work(tegra->wq, &tegra->work);
/* -- ZTE: cuijian@20111101 */	
}

/* ++ ZTE: cuijian@20111122 */   
bool check_is_otg_mode(void)
{
    if (tegra_clone)
        return tegra_clone->otg.state == OTG_STATE_A_HOST;
    else
        return false;
}
/* -- ZTE: cuijian@20111122 */  

/* ++ ZTE: lixingyuan@20110811 */   
//#include <mach/debug.h>

extern int fsl_charger_detect(void);
#ifdef CONFIG_MACH_CARDHU
extern void dc_present_start(void); //add by zhanming 20111115
#endif
static void charger_detect_work(struct work_struct *work)
{
 	struct delayed_work *delay_work = 
		container_of(work, struct delayed_work, work);
    struct tegra_otg_data *tegra =
		container_of(delay_work, struct tegra_otg_data, charger_work);
    int state;
    unsigned long int_status = tegra->int_status;
    unsigned long flags; 	
    
    printk(KERN_INFO "charger_detect_work\n"); 
    wake_lock(&tegra->vbus_lock);
    spin_lock_irqsave( &tegra->charger_detect_lock, flags);
    #ifdef CONFIG_MACH_CARDHU
    dc_present_start(); //add by zhanming 20111115
    #endif
    int_status = otg_readl(tegra, USB_PHY_WAKEUP);
    if (int_status & USB_VBUS_STATUS) {
        printk(KERN_INFO "charger_detect_work USB VBUS ON\n");
        state = fsl_charger_detect();
        if (state == 0) {
            printk(KERN_INFO "charger_detect_work USB Host\n");
        } else if (state == 1) {
            printk(KERN_INFO "charger_detect_work Charger!\n");
        } else {
            printk(KERN_INFO "charger_detect_work Unknown!\n");
        }
    } else {
        printk(KERN_INFO "charger_detect_work USB VBUS OFF\n");
        wake_unlock(&tegra->vbus_lock);
    }
    spin_unlock_irqrestore( &tegra->charger_detect_lock, flags);
}
/* -- ZTE: lixingyuan@20110811 */   

static irqreturn_t tegra_otg_irq(int irq, void *data)
{
	struct tegra_otg_data *tegra = data;
	unsigned long flags;
	unsigned long val;
/* ++ ZTE: cuijian@20111102 */
	struct tegra_otg_irqnode *otg_irq;
	//printk(KERN_INFO "tegra_otg_irq:Enter");
/* -- ZTE: cuijian@20111102 */
	spin_lock_irqsave(&tegra->lock, flags);

	val = otg_readl(tegra, USB_PHY_WAKEUP);
	if (val & (USB_VBUS_INT_EN | USB_ID_INT_EN)) {
		otg_writel(tegra, val, USB_PHY_WAKEUP);
		if ((val & USB_ID_INT_STATUS) || (val & USB_VBUS_INT_STATUS)) {
			tegra->int_status = val;
			tegra->detect_vbus = false;
/* ++ ZTE: cuijian@20111101 */	
			otg_irq = kmalloc(sizeof(struct tegra_otg_irqnode), GFP_ATOMIC);
			otg_irq->cur_int_status =val;
			list_add_tail(&otg_irq->list, &tegra->irq_queue);
			queue_work(tegra->wq, &tegra->work);
/* -- ZTE: cuijian@20111101 */				
		}
        }

/* ++ ZTE: lixingyuan@20110811 */   
       //printk(KERN_DEBUG "tegra_otg_irq:tegra->int_status 0x%08x", val);
	if (val & USB_VBUS_INT_STATUS) {
/* -- ZTE: cuijian@20111102 */
		if (val & USB_VBUS_STATUS)
			wake_lock(&tegra->vbus_lock);
/* ++ ZTE: cuijian@20111102 */
		schedule_delayed_work(&tegra->charger_work, HZ/2);
	}
/* -- ZTE: lixingyuan@20110811 */

	spin_unlock_irqrestore(&tegra->lock, flags);

	return IRQ_HANDLED;
}

void tegra_otg_check_vbus_detection(void)
{
#if 0	
	tegra_clone->detect_vbus = true;
	/* ++ ZTE: cuijian@20111101 */	
	queue_work(tegra_clone->wq, &tegra_clone->work);
	/* -- ZTE: cuijian@20111101 */	
#endif	
	tegra_otg_enable_clk();
}
EXPORT_SYMBOL(tegra_otg_check_vbus_detection);

static int tegra_otg_set_peripheral(struct otg_transceiver *otg,
				struct usb_gadget *gadget)
{
	struct tegra_otg_data *tegra;
	unsigned long val;
	/* ++ ZTE: cuijian@20111102 */	
	struct tegra_otg_irqnode *otg_irq;
	/* --ZTE: cuijian@20111102 */	

	tegra = container_of(otg, struct tegra_otg_data, otg);
	otg->gadget = gadget;

	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	val |= (USB_VBUS_INT_EN | USB_VBUS_WAKEUP_EN);
	val |= (USB_ID_INT_EN | USB_ID_PIN_WAKEUP_EN);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	/* Add delay to make sure register is updated */
	udelay(1);
	clk_disable(tegra->clk);

	if ((val & USB_ID_STATUS) && (val & USB_VBUS_STATUS)) {
		val |= USB_VBUS_INT_STATUS;
	} else if (!(val & USB_ID_STATUS)) {
		val |= USB_ID_INT_STATUS;
	} else {
		val &= ~(USB_ID_INT_STATUS | USB_VBUS_INT_STATUS);
	}

	if ((val & USB_ID_INT_STATUS) || (val & USB_VBUS_INT_STATUS)) {
		tegra->int_status = val;
		tegra->detect_vbus = false;
		/* ++ ZTE: cuijian@20111102 */	
		otg_irq = kmalloc(sizeof(struct tegra_otg_irqnode), GFP_KERNEL);
		otg_irq->cur_int_status =tegra->int_status;
		list_add_tail(&otg_irq->list, &tegra->irq_queue);
		queue_work(tegra->wq, &tegra->work);
		/* -- ZTE: cuijian@20111102 */			
	}

	return 0;
}

static int tegra_otg_set_host(struct otg_transceiver *otg,
				struct usb_bus *host)
{
	struct tegra_otg_data *tegra;
	unsigned long val;

	tegra = container_of(otg, struct tegra_otg_data, otg);
	otg->host = host;

	clk_enable(tegra->clk);
	val = otg_readl(tegra, USB_PHY_WAKEUP);
	val &= ~(USB_VBUS_INT_STATUS | USB_ID_INT_STATUS);

	val |= (USB_ID_INT_EN | USB_ID_PIN_WAKEUP_EN);
	otg_writel(tegra, val, USB_PHY_WAKEUP);
	clk_disable(tegra->clk);

	return 0;
}

static int tegra_otg_set_power(struct otg_transceiver *otg, unsigned mA)
{
	return 0;
}

static int tegra_otg_set_suspend(struct otg_transceiver *otg, int suspend)
{
	return 0;
}
/* ++ ZTE: cuijian@20111219 */	
static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct tegra_otg_data *tegra = platform_get_drvdata(pdev);
	return sprintf(buf, "%s\n", tegra_state_name(tegra->otg.state));
}
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
/* -- ZTE: cuijian@20111219 */

static int tegra_otg_probe(struct platform_device *pdev)
{
	struct tegra_otg_data *tegra;
	struct resource *res;
	int err;

	tegra = kzalloc(sizeof(struct tegra_otg_data), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	tegra->otg.dev = &pdev->dev;
	tegra->otg.label = "tegra-otg";
	tegra->otg.state = OTG_STATE_UNDEFINED;
	tegra->otg.set_host = tegra_otg_set_host;
	tegra->otg.set_peripheral = tegra_otg_set_peripheral;
	tegra->otg.set_suspend = tegra_otg_set_suspend;
	tegra->otg.set_power = tegra_otg_set_power;
	spin_lock_init(&tegra->lock);
/* ++ ZTE: cuijian@20111102 */	
	wake_lock_init(&tegra->vbus_lock, WAKE_LOCK_SUSPEND, "vbus_lock");
/* -- ZTE: cuijian@20111102 */

	platform_set_drvdata(pdev, tegra);
/* ++ ZTE: cuijian@20111219 */	
	err = device_create_file(&pdev->dev, &dev_attr_state);
/* -- ZTE: cuijian@20111219 */	

	tegra->clk_enabled = false;

	tegra->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(tegra->clk)) {
		dev_err(&pdev->dev, "Can't get otg clock\n");
		err = PTR_ERR(tegra->clk);
		goto err_clk;
	}

	err = clk_enable(tegra->clk);
	if (err)
		goto err_clken;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get I/O memory\n");
		err = -ENXIO;
		goto err_io;
	}
	tegra->regs = ioremap(res->start, resource_size(res));
	if (!tegra->regs) {
		err = -ENOMEM;
		goto err_io;
	}

	tegra->otg.state = OTG_STATE_A_SUSPEND;

	err = otg_set_transceiver(&tegra->otg);
	if (err) {
		dev_err(&pdev->dev, "can't register transceiver (%d)\n", err);
		goto err_otg;
	}

/* ++ ZTE: cuijian@20111101 */
	INIT_LIST_HEAD(&tegra->irq_queue);
	tegra->wq = alloc_workqueue("tegra_otg",
				    WQ_HIGHPRI | WQ_UNBOUND | WQ_RESCUER, 1);
	if (!tegra->wq) {
		err = -ENOMEM;
		goto err_irq;
	}
/* -- ZTE: cuijian@20111101 */	

	INIT_WORK (&tegra->work, irq_work);

/* ++ ZTE: lixingyuan@20110811 */
	spin_lock_init(&tegra->charger_detect_lock);
	INIT_DELAYED_WORK( &tegra->charger_work,  charger_detect_work);
	schedule_delayed_work(&tegra->charger_work, HZ * 7);
/* -- ZTE: lixingyuan@20110811 */

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		err = -ENXIO;
		goto err_irq;
	}
	tegra->irq = res->start;
	err = request_threaded_irq(tegra->irq, tegra_otg_irq,
				   NULL,
				   IRQF_SHARED, "tegra-otg", tegra);
	if (err) {
		dev_err(&pdev->dev, "Failed to register IRQ\n");
		goto err_irq;
	}

	tegra_clone = tegra;

#ifndef CONFIG_USB_HOTPLUG
	clk_disable(tegra->clk);
#endif

	dev_info(&pdev->dev, "otg transceiver registered\n");
	return 0;

err_irq:
	otg_set_transceiver(NULL);
err_otg:
	iounmap(tegra->regs);
err_io:
	clk_disable(tegra->clk);
err_clken:
	clk_put(tegra->clk);
err_clk:
	platform_set_drvdata(pdev, NULL);
	kfree(tegra);
	return err;
}

static int __exit tegra_otg_remove(struct platform_device *pdev)
{
	struct tegra_otg_data *tegra = platform_get_drvdata(pdev);

	free_irq(tegra->irq, tegra);
	otg_set_transceiver(NULL);
	iounmap(tegra->regs);
	clk_disable(tegra->clk);
	clk_put(tegra->clk);
	platform_set_drvdata(pdev, NULL);
	kfree(tegra);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_otg_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg_data *tegra_otg = platform_get_drvdata(pdev);
	struct otg_transceiver *otg = &tegra_otg->otg;
	enum usb_otg_state from = otg->state;
	/* store the interupt enable for cable ID and VBUS */
	clk_enable(tegra_otg->clk);
	tegra_otg->intr_reg_data = readl(tegra_otg->regs + USB_PHY_WAKEUP);
	clk_disable(tegra_otg->clk);

	if (from == OTG_STATE_B_PERIPHERAL && otg->gadget) {
		usb_gadget_vbus_disconnect(otg->gadget);
		otg->state = OTG_STATE_A_SUSPEND;
	}
	tegra_otg_disable_clk();
	return 0;
}

static void tegra_otg_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tegra_otg_data *tegra_otg = platform_get_drvdata(pdev);
	int val;
	unsigned long flags;
	/* ++ ZTE: cuijian@20111102 */	
	struct tegra_otg_irqnode *otg_irq;
	/* -- ZTE: cuijian@20111102 */	

	tegra_otg_enable_clk();

	/* Following delay is intentional.
	 * It is placed here after observing system hang.
	 * Root cause is not confirmed.
	 */
	msleep(1);
	/* restore the interupt enable for cable ID and VBUS */
	clk_enable(tegra_otg->clk);
	writel(tegra_otg->intr_reg_data, (tegra_otg->regs + USB_PHY_WAKEUP));
	val = readl(tegra_otg->regs + USB_PHY_WAKEUP);
	clk_disable(tegra_otg->clk);

	/* A device might be connected while CPU is in sleep mode. In this case no interrupt
	 * will be triggered
	 * force irq_work to recheck connected devices
	 */
	if (!(val & USB_ID_STATUS)) {
/* ++ ZTE: cuijian@20111102 */
		otg_irq = kmalloc(sizeof(struct tegra_otg_irqnode), GFP_ATOMIC);
            spin_lock_irqsave(&tegra_otg->lock, flags);
		tegra_otg->int_status = (val | USB_ID_INT_STATUS );  
		otg_irq->cur_int_status =tegra_otg->int_status;
		list_add_tail(&otg_irq->list, &tegra_otg->irq_queue);
		queue_work(tegra_otg->wq, &tegra_otg->work);
/* -- ZTE: cuijian@20111102 */		
		spin_unlock_irqrestore(&tegra_otg->lock, flags);
	}
	return;
}

static const struct dev_pm_ops tegra_otg_pm_ops = {
	.complete = tegra_otg_resume,
	.suspend = tegra_otg_suspend,
};
#endif

static struct platform_driver tegra_otg_driver = {
	.driver = {
		.name  = "tegra-otg",
#ifdef CONFIG_PM
		.pm    = &tegra_otg_pm_ops,
#endif
	},
	.remove  = __exit_p(tegra_otg_remove),
	.probe   = tegra_otg_probe,
};

static int __init tegra_otg_init(void)
{
	return platform_driver_register(&tegra_otg_driver);
}
subsys_initcall(tegra_otg_init);

static void __exit tegra_otg_exit(void)
{
	platform_driver_unregister(&tegra_otg_driver);
}
module_exit(tegra_otg_exit);
