/*
 * host_platform.c
 *
 * Copyright(c) 1998 - 2009 Texas Instruments. All rights reserved.      
 * All rights reserved.                                                  
 *                                                                       
 * Redistribution and use in source and binary forms, with or without    
 * modification, are permitted provided that the following conditions    
 * are met:                                                              
 *                                                                       
 *  * Redistributions of source code must retain the above copyright     
 *    notice, this list of conditions and the following disclaimer.      
 *  * Redistributions in binary form must reproduce the above copyright  
 *    notice, this list of conditions and the following disclaimer in    
 *    the documentation and/or other materials provided with the         
 *    distribution.                                                      
 *  * Neither the name Texas Instruments nor the names of its            
 *    contributors may be used to endorse or promote products derived    
 *    from this software without specific prior written permission.      
 *                                                                       
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "tidef.h"
#include <linux/kernel.h>
#include <asm/io.h>
#include <mach/tc.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wifi_tiwlan.h>

#include "host_platform.h"
#include "ioctl_init.h"
#include "WlanDrvIf.h"
#include "Device1273.h"

#define OS_API_MEM_ADDR		0x0000000
#define OS_API_REG_ADDR		0x0300000

static struct wifi_platform_data *wifi_control_data = NULL;
static struct resource *wifi_irqres = NULL;

static int wifi_probe( struct platform_device *pdev )
{
	struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

	printk("%s\n", __FUNCTION__);
	wifi_irqres = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "device_wifi_irq");
	if (wifi_irqres) {
		printk("wifi_irqres->start = %lu\n", (unsigned long)(wifi_irqres->start));
		printk("wifi_irqres->flags = %lx\n", wifi_irqres->flags);
	}

	if( wifi_ctrl ) {
		wifi_control_data = wifi_ctrl;
		if( wifi_ctrl->set_power )
			wifi_ctrl->set_power(1);	/* Power On */
		if( wifi_ctrl->set_reset )
			wifi_ctrl->set_reset(0);	/* Reset clear */
		if( wifi_ctrl->set_carddetect )
			wifi_ctrl->set_carddetect(1);	/* CardDetect (0->1) */
	}
	return 0;
}
											
static int wifi_remove( struct platform_device *pdev )
{
	struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

	printk("%s\n", __FUNCTION__);
	if( wifi_ctrl ) {
		if( wifi_ctrl->set_carddetect )
			wifi_ctrl->set_carddetect(0);	/* CardDetect (1->0) */
		if( wifi_ctrl->set_reset )
			wifi_ctrl->set_reset(1);	/* Reset active */
		if( wifi_ctrl->set_power )
			wifi_ctrl->set_power(0);	/* Power Off */
	}
	return 0;
}

static struct platform_driver wifi_device = {
	.probe          = wifi_probe,
	.remove         = wifi_remove,
	.suspend        = NULL,
	.resume         = NULL,
	.driver         = {
		.name   = "device_wifi",
	},
};

static int wifi_add_dev( void )
{
	printk("%s\n", __FUNCTION__);
	return platform_driver_register( &wifi_device );
}

static void wifi_del_dev( void )
{
	printk("%s\n", __FUNCTION__);
	platform_driver_unregister( &wifi_device );
}

int wifi_set_power( int on )
{
	printk("%s\n", __FUNCTION__);
	if( wifi_control_data && wifi_control_data->set_power ) {
		wifi_control_data->set_power(on);
	}
	return 0;
}

int wifi_set_reset( int on )
{
	printk("%s\n", __FUNCTION__);
	if( wifi_control_data && wifi_control_data->set_reset ) {
		wifi_control_data->set_reset(on);
	}
	return 0;
}

#if 0 /* needed for first time new host ramp*/ 
static void dump_omap_registers(void);
#endif
static void pad_config(unsigned long pad_addr, u32 andmask, u32 ormask)
{
	int val;
	u32 *addr;

	addr = (u32 *) ioremap(pad_addr, 4);
	if (!addr) {
		printk(KERN_ERR "OMAP3430_pad_config: ioremap failed with addr %lx\n", pad_addr);
		return;
	}

	val =  __raw_readl(addr);
	val &= andmask;
	val |= ormask;
	__raw_writel(val, addr);

	iounmap(addr);
}

/*-----------------------------------------------------------------------------

Routine Name:

        hPlatform_hardResetTnetw

Routine Description:

        set the GPIO to low after awaking the TNET from ELP.

Arguments:

        OsContext - our adapter context.

Return Value:

        None

-----------------------------------------------------------------------------*/

int hPlatform_hardResetTnetw( void )
{
	int err;

	/* Turn power OFF */
	if ((err = wifi_set_power(0)) == 0) {
		mdelay(500);
		/* Turn power ON*/
		err = wifi_set_power(1);
		mdelay(50);
	}
	return err;
} /* hPlatform_hardResetTnetw() */

/* Turn device power off */
int hPlatform_DevicePowerOff( void )
{
	int err;

	err = wifi_set_power(0);
	mdelay(10);
	return err;
}

/* Turn device power on */
int hPlatform_DevicePowerOn( void )
{
	int err;

	err = wifi_set_power(1);
	/* Should not be changed, 50 msec cause failures */
	mdelay(70);
	return err;
}

/*--------------------------------------------------------------------------------------*/

int hPlatform_Wlan_Hardware_Init(void *tnet_drv)
{
	TWlanDrvIfObj *drv = tnet_drv;

	printk("%s\n", __FUNCTION__);
	/* choose gpio 101, pull up */
	/* Setting MUX Mode 4 , Pull bits 0 */
	/* Should set (x is don't change):	xxxx xxxx xxxx xxxx xxxx xxxx xxx1 1000 */
	pad_config(CONTROL_PADCONF_CAM_D1, 0xFFE0FFFF, 0x001C0000);

	/* choose gpio 162, pull up, activated */
	/* Setting MUX Mode 4 , Pull bits 3 */
	/* Should set (x is don't change):	xxxx xxxx xxxx xxxx xxxx xxxx xxx1 1100 */
	pad_config(CONTROL_PADCONF_MCBSP1_CLKX, 0xFFFFFFF0, 0x0000011C);
	
	/*
	  * set pull up on all SDIO lines
	  * Setting MUX Mode of 0, and pull bits to 3
	  */

	/* set for mmc2_cmd - second half of the padconf register
	  * Should set (x is don't change):  xxxx xxxx xxx1 1000 xxxx xxxx xxxx xxxx */
	pad_config(CONTROL_PADCONF_MMC3_CMD, 0xFFFFFFF0, 0x0000011B);

	pad_config(CONTROL_PADCONF_MMC3_CLK, 0xFFF0FFE0,0x001C011A);

	
	/* set for mmc3_dat0 and dat1 - both parts of the padconf register
	  * Should set (x is don't change):  xxxx xxxx xxx1 1000 xxxx xxxx xxx1 1000 */
	pad_config(CONTROL_PADCONF_MMC3_DAT0, 0xFFF0FFF0, 0x011A011A);

	pad_config(CONTROL_PADCONF_MMC3_DAT2, 0xFFFFFFF0, 0x0000011A);

	pad_config(CONTROL_PADCONF_MMC3_DAT3, 0xFFF0FFFF, 0x011A0000);
	
#define CONTROL_PADCONF_MMC2_DAT4       0x48002164    /* set AE4 to mmc2_dat4  set AH3 to mmc2_dat5 */
	pad_config(CONTROL_PADCONF_MMC2_DAT4, 0xFFF0FFF0, 0x00180018);
	
#define CONTROL_PADCONF_MMC2_DAT6       0x48002168    /* set AF3 to mmc2_dat6  set AE3 to mmc2_dat7 */
	pad_config(CONTROL_PADCONF_MMC2_DAT6, 0xFFF0FFF0, 0x00180018);
#if 0 /* needed for first time new host ramp*/
	dump_omap_registers();
#endif
	wifi_add_dev();
	if (wifi_irqres) {
		drv->irq = wifi_irqres->start;
		drv->irq_flags = wifi_irqres->flags & IRQF_TRIGGER_MASK;
	}
	else {
		drv->irq = TNETW_IRQ;
		drv->irq_flags = (unsigned long)IRQF_TRIGGER_FALLING;
	}
	printk("%s: After wifi_add_dev()\n", __func__);
	return 0;
}

/*-----------------------------------------------------------------------------

Routine Name:

        InitInterrupt

Routine Description:

        this function init the interrupt to the Wlan ISR routine.

Arguments:

        tnet_drv - Golbal Tnet driver pointer.

Return Value:

        status

-----------------------------------------------------------------------------*/

int hPlatform_initInterrupt( void *tnet_drv, void* handle_add )
{
	TWlanDrvIfObj *drv = tnet_drv;
	int rc;
	
	if (drv->irq == 0 || handle_add == NULL)
	{
		print_err("hPlatform_initInterrupt() bad param drv->irq=%d handle_add=0x%x !!!\n",drv->irq,(int)handle_add);
		return -EINVAL;
	}
	printk("drv->irq = %u, %lx\n", drv->irq, drv->irq_flags);
	if ((rc = request_irq(drv->irq, handle_add, drv->irq_flags, drv->netdev->name, drv)))
	{
		print_err("TIWLAN: Failed to register interrupt handler\n");
		return rc;
	}
	return rc;

} /* hPlatform_initInterrupt() */

/*--------------------------------------------------------------------------------------*/

void hPlatform_freeInterrupt( void *tnet_drv )
{
	TWlanDrvIfObj *drv = tnet_drv;

	free_irq(drv->irq, drv);
}

/****************************************************************************************
 *                        hPlatform_hwGetRegistersAddr()                                 
 ****************************************************************************************
DESCRIPTION:	

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
void *hPlatform_hwGetRegistersAddr(TI_HANDLE OsContext)
{
	return (void *)OS_API_REG_ADDR;
}

/****************************************************************************************
 *                        hPlatform_hwGetMemoryAddr()                                 
 ****************************************************************************************
DESCRIPTION:	

ARGUMENTS:		

RETURN:			

NOTES:         	
*****************************************************************************************/
void *hPlatform_hwGetMemoryAddr(TI_HANDLE OsContext)
{
	return (void *)OS_API_MEM_ADDR;
}


void hPlatform_Wlan_Hardware_DeInit(void)
{
	wifi_del_dev();
}

#if 0/* needed for first time new host ramp*/
static void dump_omap_registers(void)
{
	printk(KERN_ERR "AE10 which is 0x%x= 0x%x\n", CONTROL_PADCONF_MMC3_CMD, omap_readl( CONTROL_PADCONF_MMC3_CMD ));
	printk(KERN_ERR "AC3 which is addr 0x480021D0=%x\n", omap_readl( 0x480021D0 ));

	printk(KERN_ERR "DAT0 addr 0x%x value is =%x\n", CONTROL_PADCONF_MMC3_DAT0, omap_readl( CONTROL_PADCONF_MMC3_DAT0 ));
	printk(KERN_ERR "DAT2 addr 0x%x value is =%x\n", CONTROL_PADCONF_MMC3_DAT2, omap_readl( CONTROL_PADCONF_MMC3_DAT2 ));
	printk(KERN_ERR "DAT3 addr 0x%x value is =%x\n", CONTROL_PADCONF_MMC3_DAT3, omap_readl( CONTROL_PADCONF_MMC3_DAT3 ));
	printk(KERN_ERR "DAT4 addr 0x%x value is =%x\n", CONTROL_PADCONF_MMC2_DAT4, omap_readl( CONTROL_PADCONF_MMC2_DAT4 ));
	printk(KERN_ERR "DAT6 addr 0x%x value is =%x\n", CONTROL_PADCONF_MMC2_DAT6, omap_readl( CONTROL_PADCONF_MMC2_DAT6 ));
	printk(KERN_ERR "CAM_D1 addr 0x%x value is =%x\n", CONTROL_PADCONF_CAM_D1, omap_readl( CONTROL_PADCONF_CAM_D1 ));
	printk(KERN_ERR "MCBSP1_CLKX addr 0x%x value is =%x\n", CONTROL_PADCONF_MCBSP1_CLKX, omap_readl( CONTROL_PADCONF_MCBSP1_CLKX ));
	printk(KERN_ERR "CMD addr 0x%x value is =%x\n", CONTROL_PADCONF_MMC3_CMD, omap_readl( CONTROL_PADCONF_MMC3_CMD ));
	printk(KERN_ERR "MCBSP1_CLKX addr 0x%x value is =%x\n", CONTROL_PADCONF_MCBSP1_CLKX, omap_readl( CONTROL_PADCONF_MCBSP1_CLKX ));
	printk(KERN_ERR "CLK MCBSP1_CLKX addr 0x%x value is =%x\n", CONTROL_PADCONF_MMC3_CLK, omap_readl( CONTROL_PADCONF_MMC3_CLK ));
	printk(KERN_ERR "0x480021E0 value is =%x\n", omap_readl( 0x480021E0 ));
	return;
}
#endif
