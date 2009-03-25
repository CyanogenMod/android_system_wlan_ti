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

/*-------------------------------------------------------------------*/
extern int tiwlan_sdio_init(struct sdio_func *func);
extern int sdio_reset_comm(struct mmc_card *card);

/*-------------------------------------------------------------------*/
static struct sdio_func *tiwlan_func = NULL;
static int sdio_reset_flag = 0;

#define DMA_THRESHOLD_SIZE  64
static void *sdio_dma_ptr = NULL;

/*-------------------------------------------------------------------*/
void SDIO_SetFunc( struct sdio_func *func )
{
	tiwlan_func = func;
}

struct sdio_func *SDIO_GetFunc( void )
{
	return tiwlan_func;
}

SDIO_Status SDIO_Init(SDIO_ConfigParams *ConfigParams, SDIO_Handle *Handle)
{
	if (Handle == NULL) {
		printk(KERN_ERR "Error: SDIO_Init() called with NULL!\n");
		return SDIO_FAILURE;
	}

	*Handle = (SDIO_Handle)SDIO_GetFunc();
	if ((*Handle) == NULL) {
		printk(KERN_ERR "SDIO_Init() called before init!\n");
		return SDIO_FAILURE;
	}

	if (!sdio_dma_ptr) {
		if (!(sdio_dma_ptr = kmalloc(PAGE_SIZE, GFP_KERNEL))) {
			printk(KERN_ERR "Failed to alloc DMA bounce buffer\n");
			return SDIO_FAILURE;
		}
	}
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Shutdown(SDIO_Handle Handle)
{
	if (sdio_dma_ptr) {
		kfree(sdio_dma_ptr);
		sdio_dma_ptr = NULL;
	}
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Start(SDIO_Handle Handle)
{
	struct sdio_func *func = (struct sdio_func *)Handle;

	if (func) {
		if (sdio_reset_flag) {
			sdio_reset_flag = 0;
			if (tiwlan_sdio_init(func)) {
				printk("TI: tiwlan_sdio_init Error!\n");
				return SDIO_FAILURE;
			}

		}
	}
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Reset(SDIO_Handle Handle)
{
	struct sdio_func *func = (struct sdio_func *)Handle;

	if(func && func->card)
		sdio_reset_comm(func->card);
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Stop(SDIO_Handle Handle, unsigned long Wait_Window)
{
	sdio_reset_flag = 1;
	return SDIO_Reset(Handle);
}

SDIO_Status SDIO_SyncRead(SDIO_Handle Handle, SDIO_Request_t *Req)
{
	struct sdio_func *func = (struct sdio_func *)Handle;
	int rc;

	if (Req->buffer_len < DMA_THRESHOLD_SIZE) {
		rc = sdio_memcpy_fromio(func, Req->buffer,
					Req->peripheral_addr, Req->buffer_len);
        } else {
		rc = sdio_memcpy_fromio(func, sdio_dma_ptr,
					Req->peripheral_addr, Req->buffer_len);
		memcpy(Req->buffer, sdio_dma_ptr, Req->buffer_len);
	}

	if (!rc)
		return SDIO_SUCCESS;

	printk("SDIO Write failure (%d)\n", rc);
	return SDIO_FAILURE;
}

SDIO_Status SDIO_SyncWrite(SDIO_Handle Handle, SDIO_Request_t *Req)
{
	struct sdio_func *func = (struct sdio_func *)Handle;
	int rc;
	void *dma_ptr;

	if (Req->buffer_len < DMA_THRESHOLD_SIZE)
		dma_ptr = Req->buffer;
	else {
		dma_ptr = sdio_dma_ptr;
		memcpy(dma_ptr, Req->buffer, Req->buffer_len);
	}

	rc = sdio_memcpy_toio(func, Req->peripheral_addr, dma_ptr,
			      Req->buffer_len);
	if (!rc)
		return SDIO_SUCCESS;

	printk("SDIO Write failure (%d)\n", rc);
	return SDIO_FAILURE;
}
#endif
