/*
 * RxXfer.c
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


/****************************************************************************
 *
 *   MODULE:  rxXfer.c
 *
 *   PURPOSE: Rx Xfer module implementation.Responsible for reading Rx from the FW
 *              and forward it to the upper layers.
 * 
 ****************************************************************************/

#define __FILE_ID__  FILE_ID_106
#include "tidef.h"
#include "osApi.h"
#include "report.h"
#include "RxXfer.h"
#include "FwEvent_api.h"
#include "TWDriverInternal.h"
#include "RxQueue_api.h"
#include "TwIf.h"
#include "public_host_int.h"
#include "bmtrace_api.h"

#define RX_DRIVER_COUNTER_ADDRESS 0x300538
#define RX_DRIVER_DUMMY_WRITE_ADDRESS 0x300534
#define PLCP_HEADER_LENGTH 8
#define WORD_SIZE   4
#define UNALIGNED_PAYLOAD   0x1

#define SLV_MEM_ADDR_VALUE(desc, offset)((RX_DESC_GET_MEM_BLK(desc)<<8)+ offset + 4)
#define SLV_MEM_CP_VALUE(desc, offset)  (((RX_DESC_GET_MEM_BLK(desc)<<8)+ offset))
/* Add an extra word for alignment the MAC payload in case of QoS MSDU */
#define ALIGNMENT_SIZE(desc)            ((RX_DESC_GET_UNALIGNED(desc) & UNALIGNED_PAYLOAD)? 2 : 0)


/************************ static function declaration *****************************/

static TI_STATUS rxXfer_Handle(TI_HANDLE hRxXfer);
static void rxXfer_TxnDoneCb (TI_HANDLE hRxXfer, TTxnStruct* pTxn);
static void rxXfer_IssueTxn (TI_HANDLE hRxXfer, TI_UINT32 uRxDesc, TI_UINT8 *pHostBuf, TI_UINT32 uBuffSize, TI_BOOL bDropPacket);
static void rxXfer_ForwardPacket (RxXfer_t* pRxXfer, TTxnStruct* pTxn);

/****************************************************************************
 *                      RxXfer_Create()
 ****************************************************************************
 * DESCRIPTION: Create the RxXfer module object 
 * 
 * INPUTS:  None
 * 
 * OUTPUT:  None
 * 
 * RETURNS: The Created object
 ****************************************************************************/
TI_HANDLE rxXfer_Create (TI_HANDLE hOs)
{
    RxXfer_t *pRxXfer;

    pRxXfer = os_memoryAlloc (hOs, sizeof(RxXfer_t));
    if (pRxXfer == NULL)
        return NULL;

    /* For all the counters */
    os_memoryZero (hOs, pRxXfer, sizeof(RxXfer_t));

    pRxXfer->hOs = hOs;

    return (TI_HANDLE)pRxXfer;
}


/****************************************************************************
 *                      RxXfer_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the RxXfer module object 
 * 
 * INPUTS:  hRxXfer - The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/
void rxXfer_Destroy (TI_HANDLE hRxXfer)
{
    RxXfer_t *pRxXfer = (RxXfer_t *)hRxXfer;

    if (pRxXfer)
    {
        os_memoryFree (pRxXfer->hOs, pRxXfer, sizeof(RxXfer_t));
    }
}


/****************************************************************************
 *                      rxXfer_init()
 ****************************************************************************
 * DESCRIPTION: Init the FwEvent module object 
 * 
 * INPUTS:      hRxXfer       - FwEvent handle;
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void rxXfer_Init(TI_HANDLE hRxXfer,
                 TI_HANDLE hFwEvent, 
                 TI_HANDLE hReport,
                 TI_HANDLE hTwIf,
                 TI_HANDLE hRxQueue)
{
    RxXfer_t  *pRxXfer      = (RxXfer_t *)hRxXfer;

    pRxXfer->hFwEvent       = hFwEvent;
    pRxXfer->hReport        = hReport;
    pRxXfer->hTwIf          = hTwIf;
    pRxXfer->hRxQueue       = hRxQueue;
    pRxXfer->uDrvRxCntr     = 0;

    RxXfer_ReStart (hRxXfer, TI_TRUE);

#ifdef TI_DBG   
    rxXfer_ClearStats (pRxXfer);
#endif
}


/****************************************************************************
 *                      rxXfer_Register_CB()
 ****************************************************************************
 * DESCRIPTION: Register the function to be called for request for buffer.
 * 
 * INPUTS:      hRxXfer       - RxXfer handle;
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void rxXfer_Register_CB (TI_HANDLE hRxXfer, TI_UINT32 CallBackID, void *CBFunc, TI_HANDLE CBObj)
{
    RxXfer_t* pRxXfer = (RxXfer_t *)hRxXfer;

    TRACE1(pRxXfer->hReport, REPORT_SEVERITY_INFORMATION , "rxXfer_Register_CB (Value = 0x%x)\n", CallBackID);

    switch(CallBackID)
    {
    case TWD_INT_REQUEST_FOR_BUFFER:       
        pRxXfer->RequestForBufferCB = (TRequestForBufferCb)CBFunc;
        pRxXfer->RequestForBufferCB_handle = CBObj;
        break;

    default:
        TRACE0(pRxXfer->hReport, REPORT_SEVERITY_ERROR, "rxXfer_Register_CB - Illegal value\n");
        return;
    }
}


/****************************************************************************
 *                      rxXfer_ForwardPacket()
 ****************************************************************************
 * DESCRIPTION:  Forward received packet to the upper layers.
 *
 * INPUTS:      
 * 
 * OUTPUT:      
 * 
 * RETURNS:     
 ****************************************************************************/
static void rxXfer_ForwardPacket (RxXfer_t* pRxXfer, TTxnStruct* pTxn)
{
#ifdef TI_DBG   /* packet sanity check */
    RxIfDescriptor_t *pRxInfo  = (RxIfDescriptor_t*)(pTxn->aBuf[0]);
    TI_UINT16        uLenFromRxInfo;
  
    /* Get length from RxInfo, handle endianess and convert to length in bytes */
    uLenFromRxInfo = ENDIAN_HANDLE_WORD(pRxInfo->length) << 2;

    /* If the length in the RxInfo is different than in the short descriptor, set error status */
    if (pTxn->aLen[0] != uLenFromRxInfo) 
    {
        TRACE3(pRxXfer->hReport, REPORT_SEVERITY_ERROR , ": Bad Length!! RxInfoLength=%d, ShortDescLen=%d, RxInfoStatus=0x%x\n", uLenFromRxInfo, pTxn->aLen[0], pRxInfo->status);
        report_PrintDump(pTxn->aBuf[0], pTxn->aLen[0]);

        pRxInfo->status &= ~RX_DESC_STATUS_MASK;
        pRxInfo->status |= RX_DESC_STATUS_DRIVER_RX_Q_FAIL;
        pRxInfo->length = ENDIAN_HANDLE_WORD(pTxn->aLen[0] >> 2);
    }
#endif

    /* Forward received packet to the upper layers */
    RxQueue_ReceivePacket (pRxXfer->hRxQueue, (const void *)pTxn->aBuf[0]);

    /* reset the aBuf field for clean on recovery purpose */
    pTxn->aBuf[0] = 0;
}


/****************************************************************************
 *                      rxXfer_RxEvent()
 ****************************************************************************
 * DESCRIPTION: Called upon Rx event from the FW.calls the SM  
 * 
 * INPUTS:      hRxXfer       - RxXfer handle;
 * 
 * OUTPUT:  None
 * 
 * RETURNS: TWIF_OK in case of Synch mode, or TWIF_PENDING in case of Asynch mode
 *          (when returning TWIF_PENDING, FwEvent module expects the FwEvent_EventComplete()
 *          function call to finish the Rx Client handling 
 *
 ****************************************************************************/
TI_STATUS rxXfer_RxEvent (TI_HANDLE hRxXfer, FwStatus_t *pFwStatus)
{
    RxXfer_t       *pRxXfer = (RxXfer_t *)hRxXfer;
    TI_UINT32      uTempCounters;
    FwStatCntrs_t  *pFwStatusCounters;
    TI_UINT32       i;
    TI_STATUS   rc;
    CL_TRACE_START_L2();
  
    uTempCounters = ENDIAN_HANDLE_LONG (pFwStatus->counters);
#ifdef TI_DBG
	pRxXfer->DbgStats.counters = uTempCounters;
#endif
    pFwStatusCounters = (FwStatCntrs_t*)(&uTempCounters);

    TRACE2(pRxXfer->hReport, REPORT_SEVERITY_INFORMATION , ": NewFwCntr=%d, OldFwCntr=%d\n", pFwStatusCounters->fwRxCntr, pRxXfer->uFwRxCntr);

    if (pFwStatusCounters->fwRxCntr%8 == pRxXfer->uFwRxCntr%8)
    {
        return TI_OK;
    }
    pRxXfer->uFwRxCntr = pFwStatusCounters->fwRxCntr;

    for (i = 0; i < NUM_RX_PKT_DESC; i++)
    {
        pRxXfer->aRxPktsDesc[i] = ENDIAN_HANDLE_LONG (pFwStatus->rxPktsDesc[i]); 
    }

    rc = rxXfer_Handle (pRxXfer);

    CL_TRACE_END_L2("tiwlan_drv.ko", "INHERIT", "RX", ".RxXferEvent");
    return rc;
}


/****************************************************************************
 *                      rxXfer_Handle()
 ****************************************************************************
 * DESCRIPTION: 
 *
 * INPUTS:      hRxXfer       - RxXfer handle;
 * 
 * OUTPUT:      
 * 
 * RETURNS:     
 ****************************************************************************/
static TI_STATUS rxXfer_Handle(TI_HANDLE hRxXfer)
{
#ifndef _VLCT_
    RxXfer_t* pRxXfer = (RxXfer_t *)hRxXfer;
    TI_UINT32   uRxDesc, uPacketIndex, uBuffSize;
    TI_UINT8 *pHostBuf;
    ERxBufferStatus  eBufStatus;


    if (pRxXfer->uAvailableTxn == 0 )
    {
        TRACE0(pRxXfer->hReport, REPORT_SEVERITY_ERROR, "(): No available Txn structures left!\n");
        return TI_NOK;
    }

    while (pRxXfer->uFwRxCntr%8 != pRxXfer->uDrvRxCntr%8)
    {
        uPacketIndex = pRxXfer->uDrvRxCntr % NUM_RX_PKT_DESC;

        uRxDesc =  pRxXfer->aRxPktsDesc[uPacketIndex];
        
        uBuffSize = RX_DESC_GET_LENGTH(uRxDesc) << 2;

        /* prepare the read buffer */
        /* the RxBufAlloc() add an extra word for alignment the MAC payload in case of QoS MSDU */
        eBufStatus = pRxXfer->RequestForBufferCB(pRxXfer->RequestForBufferCB_handle, 
                                                 (void**)&pHostBuf,
                                                 uBuffSize,
                                                 (TI_UINT32)NULL);

        TRACE6(pRxXfer->hReport, REPORT_SEVERITY_INFORMATION , ": Index=%d, RxDesc=0x%x, DrvCntr=%d, FwCntr=%d, BufStatus=%d, BuffSize=%d\n", uPacketIndex, uRxDesc, pRxXfer->uDrvRxCntr, pRxXfer->uFwRxCntr, eBufStatus, uBuffSize);

        switch (eBufStatus)
        {
            case RX_BUF_ALLOC_PENDING:
                return TI_OK;

            case RX_BUF_ALLOC_COMPLETE:
                rxXfer_IssueTxn (pRxXfer, uRxDesc, pHostBuf, uBuffSize, TI_FALSE);
                break;

            case RX_BUF_ALLOC_OUT_OF_MEM:
                /* In case the allocation failed, we read the packet to a temporary buffer and ignore it */
                rxXfer_IssueTxn (pRxXfer, uRxDesc, (TI_UINT8*)pRxXfer->aTempBuffer, uBuffSize, TI_TRUE);
                break;
        }

    /* End of while */
    }
#endif
    return TI_OK;
/* End of rxXfer_Handle() */
}


/****************************************************************************
 *                      rxXfer_IssueTxn()
 ****************************************************************************
 * DESCRIPTION: 
 *
 * INPUTS:      
 * 
 * OUTPUT:      
 * 
 * RETURNS:     
 ****************************************************************************/
static void rxXfer_IssueTxn (TI_HANDLE hRxXfer, TI_UINT32 uRxDesc, TI_UINT8 *pHostBuf, TI_UINT32 uBuffSize, TI_BOOL bDropPacket)
{
    RxXfer_t   *pRxXfer = (RxXfer_t *)hRxXfer;
    TI_UINT32   uIndex = pRxXfer->uDrvRxCntr % MAX_CONSECUTIVE_READ_TXN;
    TTxnStruct *pTxn;
    ETxnStatus  eStatus;

    /* Write the next mem block that we want to read */
    pRxXfer->aSlaveRegTxn[uIndex].tTxnStruct.uHwAddr = SLV_REG_DATA;
    pRxXfer->aSlaveRegTxn[uIndex].uRegData = SLV_MEM_CP_VALUE(uRxDesc, pRxXfer->uPacketMemoryPoolStart);
    pRxXfer->aSlaveRegTxn[uIndex].uRegAdata = SLV_MEM_ADDR_VALUE(uRxDesc, pRxXfer->uPacketMemoryPoolStart);
    twIf_Transact(pRxXfer->hTwIf, &pRxXfer->aSlaveRegTxn[uIndex].tTxnStruct);

    /* prepare the read transaction */ 
    pTxn = (TTxnStruct*)&pRxXfer->aTxnStruct[uIndex];

    if (!bDropPacket)
    {
        pHostBuf += ALIGNMENT_SIZE(uRxDesc);
        BUILD_TTxnStruct(pTxn, SLV_MEM_DATA, pHostBuf, uBuffSize, (TTxnDoneCb)rxXfer_TxnDoneCb, hRxXfer)
    }
    else
    {
        TRACE0(pRxXfer->hReport, REPORT_SEVERITY_ERROR, "Request for Rx buffer failed! \n");
        BUILD_TTxnStruct(pTxn, SLV_MEM_DATA, pHostBuf, uBuffSize, NULL, NULL)
    }

    eStatus = twIf_Transact(pRxXfer->hTwIf, pTxn);

    pRxXfer->uDrvRxCntr++;

    /* work around for WL6-PG1.0 is still needed for PG2.0 */
    /* if (pRxXfer->bChipIs1273Pg10) */
    {
    pTxn = &pRxXfer->aCounterTxn[uIndex].tTxnStruct;
    pRxXfer->aCounterTxn[uIndex].uCounter = ENDIAN_HANDLE_LONG(pRxXfer->uDrvRxCntr);
    TXN_PARAM_SET(pTxn, TXN_LOW_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_WRITE, TXN_INC_ADDR)
    BUILD_TTxnStruct(pTxn, RX_DRIVER_COUNTER_ADDRESS, &pRxXfer->aCounterTxn[uIndex].uCounter, REGISTER_SIZE, NULL, NULL)
    twIf_Transact(pRxXfer->hTwIf, pTxn);

    pTxn = &pRxXfer->aDummyTxn[uIndex].tTxnStruct;
    pRxXfer->aDummyTxn[uIndex].uData = 0x1;
    TXN_PARAM_SET(pTxn, TXN_LOW_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_WRITE, TXN_INC_ADDR)
    BUILD_TTxnStruct(pTxn, RX_DRIVER_DUMMY_WRITE_ADDRESS, &pRxXfer->aDummyTxn[uIndex].uData, REGISTER_SIZE, NULL, NULL)
    twIf_Transact(pRxXfer->hTwIf, pTxn);

    TRACE6(pRxXfer->hReport, REPORT_SEVERITY_INFORMATION , ": Counter-Txn: HwAddr=0x%x, Len0=%d, Data0=%d, DrvCount=%d, TxnParams=0x%x, RxDesc=0x%x\n", pTxn->uHwAddr, pTxn->aLen[0], *(TI_UINT32 *)(pTxn->aBuf[0]), pRxXfer->uDrvRxCntr, pTxn->uTxnParams, uRxDesc);
    }

    if (!bDropPacket)
    {
        if (eStatus == TXN_STATUS_COMPLETE)
        {
            /* Forward received packet to the upper layers */
            rxXfer_ForwardPacket (pRxXfer, &(pRxXfer->aTxnStruct[uIndex]));
        }
        else if (eStatus == TXN_STATUS_PENDING) 
        {
            /* Decrease the number of available txn structures */
            pRxXfer->uAvailableTxn--;
        }
        else 
        {
            TRACE3(pRxXfer->hReport, REPORT_SEVERITY_ERROR , ": Status=%d, DrvCntr=%d, RxDesc=0x%x\n", eStatus, pRxXfer->uDrvRxCntr, uRxDesc);
        }
    }
}
  

/****************************************************************************
 *                      rxXfer_SetRxDirectAccessParams()
 ****************************************************************************
 * DESCRIPTION: 
 *
 * INPUTS:      
 * 
 * OUTPUT:      
 * 
 * RETURNS:     
 ****************************************************************************/
void rxXfer_SetRxDirectAccessParams (TI_HANDLE hRxXfer, TDmaParams *pDmaParams)
{
    RxXfer_t* pRxXfer = (RxXfer_t *)hRxXfer;

    pRxXfer->uPacketMemoryPoolStart = pDmaParams->PacketMemoryPoolStart;
}


/****************************************************************************
 *                      rxXfer_TxnDoneCb()
 ****************************************************************************
 * DESCRIPTION: Forward the packet to the registered CB
 *
 * INPUTS:      
 * 
 * OUTPUT:      
 * 
 * RETURNS:     
 ****************************************************************************/
static void rxXfer_TxnDoneCb (TI_HANDLE hRxXfer, TTxnStruct* pTxn)
{
    RxXfer_t* pRxXfer = (RxXfer_t *)hRxXfer;
    CL_TRACE_START_L2();
    
    /* Increase the number of available txn structures */
    pRxXfer->uAvailableTxn++;

    /* Forward received packet to the upper layers */
    rxXfer_ForwardPacket (pRxXfer, pTxn);

    /* Handle further packets if any */
    rxXfer_Handle(hRxXfer);

    CL_TRACE_END_L2("tiwlan_drv.ko", "INHERIT", "RX", ".RxXferTxnDone");
}


/****************************************************************************
 *                      RxXfer_ReStart()
 ****************************************************************************
 * DESCRIPTION:	RxXfer_ReStart the RxXfer module object (called by the recovery)
 * 
 * INPUTS:	hRxXfer - The object to free
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	NONE 
 ****************************************************************************/
void RxXfer_ReStart(TI_HANDLE hRxXfer, TI_BOOL bChipIs1273Pg10)
{
	RxXfer_t *pRxXfer = (RxXfer_t *)hRxXfer;
    TTxnStruct* pTxn;
    TI_UINT8    i;

    /* remove the chipID check when WL6-PG1.0 becomes obsolete */
    pRxXfer->bChipIs1273Pg10 = bChipIs1273Pg10;


    pRxXfer->uFwRxCntr = 0;
    pRxXfer->uDrvRxCntr = 0;
    pRxXfer->uAvailableTxn = MAX_CONSECUTIVE_READ_TXN - 1;

    /* Scan all transaction array and release only pending transaction */
    for (i = 0; i < MAX_CONSECUTIVE_READ_TXN; i++)
    {
        pTxn = &(pRxXfer->aSlaveRegTxn[i].tTxnStruct);
        /* Check if buffer allocated and not the local one (signed by no callback) */
        if ((pTxn->hCbHandle != NULL) && (pTxn->aBuf[0] != 0))
        {
            RxIfDescriptor_t    *pRxParams  = (RxIfDescriptor_t*)pTxn->aBuf[0];

            WLAN_OS_REPORT (("RxXfer_ReStart: clean, in loop call RxQueue_ReceivePacket with TAG_CLASS_UNKNOWN\n"));
            /* Set TAG_CLASS_UNKNOWN and call upper layer only to release the allocated buffer */
            pRxParams->packet_class_tag = TAG_CLASS_UNKNOWN;
            RxQueue_ReceivePacket (pRxXfer->hRxQueue, (const void *)pTxn->aBuf[0]);
            pTxn->aBuf[0] = 0;
        }
    }

    for (i = 0; i < MAX_CONSECUTIVE_READ_TXN; i++)
    {
        pTxn = &(pRxXfer->aSlaveRegTxn[i].tTxnStruct);
        TXN_PARAM_SET(pTxn, TXN_LOW_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_WRITE, TXN_INC_ADDR)
        BUILD_TTxnStruct(pTxn, SLV_REG_DATA, &pRxXfer->aSlaveRegTxn[i].uRegData, REGISTER_SIZE*2, NULL, NULL)

        pTxn = &pRxXfer->aTxnStruct[i];
        TXN_PARAM_SET(pTxn, TXN_LOW_PRIORITY, TXN_FUNC_ID_WLAN, TXN_DIRECTION_READ, TXN_FIXED_ADDR)
    }
	
} /* RxXfer_ReStart() */


#ifdef TI_DBG
/****************************************************************************
 *                      rxXfer_ClearStats()
 ****************************************************************************
 * DESCRIPTION: 
 *
 * INPUTS:  
 *          pRxXfer The object
 * 
 * OUTPUT:  None
 * 
 * RETURNS: TI_OK. 
 ****************************************************************************/
void rxXfer_ClearStats (TI_HANDLE hRxXfer)
{
    RxXfer_t * pRxXfer = (RxXfer_t *)hRxXfer;

    os_memoryZero (pRxXfer->hOs, &pRxXfer->DbgStats, sizeof(RxXferStats_T));
}


/****************************************************************************
 *                      rxXfer_PrintStats()
 ****************************************************************************
 * DESCRIPTION: .
 *
 * INPUTS:  
 *          pRxXfer The object
 * 
 * OUTPUT:  None
 * 
 * RETURNS: TI_OK. 
 ****************************************************************************/
void rxXfer_PrintStats (TI_HANDLE hRxXfer)
{
    RxXfer_t * pRxXfer = (RxXfer_t *)hRxXfer;
    
    WLAN_OS_REPORT(("Print RX Xfer module info\n"));
    WLAN_OS_REPORT(("=========================\n"));
    WLAN_OS_REPORT(("Rx counter   = 0x%x\n", pRxXfer->uFwRxCntr));
    WLAN_OS_REPORT(("Drv counter  = 0x%x\n", pRxXfer->uDrvRxCntr));
    WLAN_OS_REPORT(("Fw Counters  = 0x%x\n", pRxXfer->DbgStats.counters));
    WLAN_OS_REPORT(("AvailableTxn = 0x%x\n", pRxXfer->uAvailableTxn));
}
#endif


