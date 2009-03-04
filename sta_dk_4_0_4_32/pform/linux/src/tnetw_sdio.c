/* tnetw_sdio.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright © Texas Instruments Incorporated (Oct 2005)
 * THIS CODE/PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDED BUT NOT LIMITED TO , THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * This program has been modified from its original operation by Texas
 * Instruments Incorporated. These changes are covered under version 2
 * of the GNU General Public License, dated June 1991.
 *
 * Copyright © Google Inc (Feb 2008)
 */
/*-------------------------------------------------------------------*/
#ifdef TIWLAN_MSM7000
#include <linux/delay.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include "esta_drv.h"
#include "mmc_omap_api.h"
#include "osApi.h"
#define DM_DMA_WORKAROUND
/*-------------------------------------------------------------------*/
extern int tiwlan_sdio_init(struct sdio_func *func);
extern int sdio_reset_comm(struct mmc_card *card);
/*-------------------------------------------------------------------*/
static struct sdio_func *tiwlan_func = NULL;
static int sdio_reset_flag = 0;
#ifdef DM_DMA_WORKAROUND
#define DMA_THRESHOLD_SIZE  64
static void *sdio_dma_ptr = NULL;
#endif
/*-------------------------------------------------------------------*/
void SDIO_SetFunc( struct sdio_func *func )
{
	tiwlan_func = func;
}

struct sdio_func *SDIO_GetFunc( void )
{
	return( tiwlan_func );
}

SDIO_Status SDIO_Init(SDIO_ConfigParams *ConfigParams, SDIO_Handle *Handle)
{
	if( Handle == NULL ) {
		printk(KERN_ERR "Error: SDIO_Init() called with NULL!\n");
		return SDIO_FAILURE;
	}
	*Handle = (SDIO_Handle)SDIO_GetFunc();
	if( (*Handle) == NULL ) {
		printk(KERN_ERR "Error: SDIO_Init() called before SDIO probe completed!\n");
		return SDIO_FAILURE;
	}
#ifdef DM_DMA_WORKAROUND
    if( !sdio_dma_ptr ) {
        sdio_dma_ptr = kmalloc( PAGE_SIZE, GFP_KERNEL );
        if( !sdio_dma_ptr )
            return SDIO_FAILURE;
    }
#endif
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Shutdown(SDIO_Handle Handle)
{
	/* printk("%s:\n", __FUNCTION__); */
#ifdef DM_DMA_WORKAROUND
    if( sdio_dma_ptr ) {
        kfree( sdio_dma_ptr );
        sdio_dma_ptr = NULL;
    }
#endif
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Start(SDIO_Handle Handle)
{
	struct sdio_func *func = (struct sdio_func *)Handle;

	/* printk("%s:\n", __FUNCTION__); */
	if( func ) {
		if( sdio_reset_flag ) {
			sdio_reset_flag = 0;
			if( tiwlan_sdio_init(func) ) {
				printk("TI: tiwlan_sdio_init Error !\n");
				return SDIO_FAILURE;
			}
		}
	}
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Reset(SDIO_Handle Handle)
{
	struct sdio_func *func = (struct sdio_func *)Handle;

	/* printk("%s:\n", __FUNCTION__); */
	if( func && func->card ) {
		sdio_reset_comm(func->card);
	}
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Stop(SDIO_Handle Handle, unsigned long Wait_Window)
{
	/* printk("%s:\n", __FUNCTION__); */
	sdio_reset_flag = 1;
	return SDIO_Reset( Handle );
}

static int read_direct(struct sdio_func *func, unsigned char *buf, 
                            unsigned long addr, unsigned len)
{
	unsigned i;
	int rc0, rc = 0;

	for(i=0;( i < len );i++,addr++) {
		*buf++ = (unsigned char)sdio_readb(func, addr, &rc0);
		if( rc0 != 0 )
			rc = rc0;
	}
	return rc;
}

static int write_direct(struct sdio_func *func, unsigned long addr,
			unsigned char *buf, unsigned len)
{
	unsigned i;
	int rc0, rc = 0;

	for(i=0;( i < len );i++,addr++) {
		sdio_writeb(func, *buf++, addr, &rc0);
		if( rc0 != 0 )
			rc = rc0;
	}
	return rc;
}

SDIO_Status SDIO_SyncRead(SDIO_Handle Handle, SDIO_Request_t *Req)
{
	struct sdio_func *func = (struct sdio_func *)Handle;
	int rc, retries = 5;

#if 0
	printk("%s: p_addr = 0x%.8lx, sz = %d\n",
	       __FUNCTION__,
	       Req->peripheral_addr,
	       Req->buffer_len);
#endif

	while(retries) {
        if( retries > 2 )
#ifdef DM_DMA_WORKAROUND
            if( Req->buffer_len < DMA_THRESHOLD_SIZE ) {
                rc = sdio_memcpy_fromio(func, Req->buffer, Req->peripheral_addr, Req->buffer_len);
            }
            else {
                rc = sdio_memcpy_fromio(func, sdio_dma_ptr, Req->peripheral_addr, Req->buffer_len);
                memcpy( Req->buffer, sdio_dma_ptr, Req->buffer_len );
            }
#else
            rc = sdio_memcpy_fromio(func, Req->buffer, Req->peripheral_addr, Req->buffer_len);
#endif
        else
            rc = read_direct(func, Req->buffer, Req->peripheral_addr, Req->buffer_len);

		if (rc) {
			printk(KERN_ERR "%s: Read operation failed (%d) (retries = %d)\n",
		       		__FUNCTION__, rc, retries);
			retries--;
			continue;
		}
		if (retries != 5)
			printk(KERN_ERR "%s: Retry succeeded\n", __FUNCTION__);
		return SDIO_SUCCESS;
	}
	printk(KERN_ERR "%s: Giving up\n", __FUNCTION__);
	return SDIO_FAILURE;
}

SDIO_Status SDIO_SyncWrite(SDIO_Handle Handle, SDIO_Request_t *Req)
{
	struct sdio_func *func = (struct sdio_func *)Handle;
	int rc, retries = 5;
#ifdef DM_DMA_WORKAROUND
    void *dma_ptr;
#endif

#if 0
	printk("%s: p_addr = 0x%.8lx, sz = %d\n",
	       __FUNCTION__,
	       Req->peripheral_addr,
	       Req->buffer_len);
#endif

	while(retries) {
        if( retries > 2 ) {
#ifdef DM_DMA_WORKAROUND
            if( Req->buffer_len < DMA_THRESHOLD_SIZE ) {
                dma_ptr = Req->buffer;
            }
            else {
                dma_ptr = sdio_dma_ptr;
                memcpy( dma_ptr, Req->buffer, Req->buffer_len );
            }
            rc = sdio_memcpy_toio(func, Req->peripheral_addr, dma_ptr, Req->buffer_len);
#else
            rc = sdio_memcpy_toio(func, Req->peripheral_addr, Req->buffer, Req->buffer_len);
#endif
        }
        else
            rc = write_direct(func, Req->peripheral_addr, Req->buffer, Req->buffer_len);

		if (rc) {
			printk(KERN_ERR "%s: Write operation failed (%d) (retries = %d)\n",
		       	__FUNCTION__, rc, retries);
			retries--;
			continue;
		}

		if (retries != 5)
			printk(KERN_ERR "%s: Retry succeeded\n", __FUNCTION__);
		return SDIO_SUCCESS;
	}
	printk(KERN_ERR "%s: Giving up\n", __FUNCTION__);
	return SDIO_FAILURE;
}
#endif
