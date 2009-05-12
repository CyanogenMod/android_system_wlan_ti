/*
 * roamingMngr.c
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

/** \file roamingMngr.c
 *  \brief Roaming Manager
 *
 *  \see roamingMngrApi.h
 */

/****************************************************************************
 *                                                                          *
 *   MODULE:  Roaming Manager                                               *
 *   PURPOSE:                                                               *
 *   Roaming manager is responsible to receive Roaming triggers and try
 *      to select a better AP.
 *      The Roaming triggers are: Low RSSI, PER, consecutive No ACK on TX,
 *      beacon Missed or External request.
 *      In each Internal Roaming request, scan is performed and selection for 
 *      better AP. Better AP is defined as a different AP with better RSSI,
 *      and similar SSID and security settings.
 *      If better AP is found, there is a check for fast-roaming via the
 *      Supplicant. Then connection to the new AP is invoked.
 *                                                                          *
 ****************************************************************************/

#define __FILE_ID__  FILE_ID_8
#include "osApi.h"

#include "paramOut.h"
#include "report.h"
#include "fsm.h"

#include "scanMngrApi.h"
#include "roamingMngrApi.h"
#include "apConnApi.h"
#include "roamingMngrTypes.h"
#include "bssTypes.h"
#include "DrvMainModules.h"
#include "TWDriver.h"
#include "siteMgrApi.h"

/*-----------*/
/* Constants */
/*-----------*/

/* Init bits */
#define ROAMING_MNGR_CONTEXT_INIT_BIT       1
#define ROAMING_MNGR_SM_INIT_BIT            2

#define DEFAULT_AP_QUALITY                  (-70)
#define DEFAULT_LOW_PASS_FILTER             (30)
#define DEFAULT_DATA_RETRY_THRESHOLD        (20)
#define DEFAULT_LOW_QUALITY_SCAN_COND       (-60)
#define DEFAULT_NORMAL_QUALITY_SCAN_COND    (-50)
#define DEFAULT_LOW_RSSI                    (-70)
#define DEFAULT_LOW_SNR                     (0)
#define DEFAULT_TBTT_4_BSS_LOSS             (10)
#define DEFAULT_LOW_TX_RATE                 (2)


#define ROAMING_MNGR_NUM_STATES     	ROAMING_STATE_LAST   
#define ROAMING_MNGR_NUM_EVENTS     	ROAMING_EVENT_LAST  

#define INVALID_CANDIDATE_INDEX     	0xFF
#define CURRENT_AP_INDEX            	0xFE


#define MAX_ROAMING_TRIGGERS  ROAMING_TRIGGER_LAST


/*--------------*/
/* Enumerations */
/*--------------*/

/** state machine states */
typedef enum 
{
/*	0	*/	ROAMING_STATE_IDLE,
/*	1	*/	ROAMING_STATE_WAIT_4_TRIGGER,
/*	2	*/  ROAMING_STATE_WAIT_4_CMD,
/*	3	*/  ROAMING_STATE_SCANNING,
/*	4	*/  ROAMING_STATE_SELECTING,
/*	5	*/  ROAMING_STATE_CONNECTING,
/*	6	*/  ROAMING_STATE_LAST

} roamingMngr_smStates;

/** State machine events */
typedef enum 
{
/*	0	*/	ROAMING_EVENT_START, 			/* CONNECTED */
/*	1	*/	ROAMING_EVENT_STOP, 			/* NOT CONNECTED */
/*	2	*/	ROAMING_EVENT_ROAM_TRIGGER,
/*	3	*/	ROAMING_EVENT_SCAN, 
/*	4	*/	ROAMING_EVENT_SELECT, 
/*	5	*/	ROAMING_EVENT_REQ_HANDOVER, 
/*	6	*/	ROAMING_EVENT_ROAM_SUCCESS, 
/*	7	*/	ROAMING_EVENT_FAILURE, 
/*	8	*/	ROAMING_EVENT_LAST

} roamingMngr_smEvents;

/* scan types */
typedef enum 
{
/*	0	*/	ROAMING_NO_SCAN, 
/*	1	*/	ROAMING_PARTIAL_SCAN,
/*	2	*/	ROAMING_PARTIAL_SCAN_RETRY,
/*	3	*/	ROAMING_FULL_SCAN,
/*	4	*/	ROAMING_FULL_SCAN_RETRY

} scan4RoamingType_e;

/* Roaming Trigger groups, according to Roaming Triggers */
typedef enum
{
    ROAMING_TRIGGER_BG_SCAN_GROUP 		= ROAMING_TRIGGER_NORMAL_QUALITY_FOR_BG_SCAN,
    ROAMING_TRIGGER_LOW_QUALITY_GROUP 	= ROAMING_TRIGGER_MAX_TX_RETRIES,
    ROAMING_TRIGGER_FAST_CONNECT_GROUP 	= ROAMING_TRIGGER_SWITCH_CHANNEL,
    ROAMING_TRIGGER_FULL_CONNECT_GROUP 	= ROAMING_TRIGGER_SECURITY_ATTACK
} roamingMngr_connectTypeGroup_e;


/*----------*/
/* Typedefs */
/*----------*/

typedef struct _roamingMngr_t   roamingMngr_t;

/*------------*/
/* Structures */
/*------------*/

typedef struct 
{
    TI_UINT8   preAuthBSSList[MAX_SIZE_OF_BSS_TRACK_LIST];
    TI_UINT8   numOfPreAuthBSS;
    TI_UINT8   neighborBSSList[MAX_SIZE_OF_BSS_TRACK_LIST];
    TI_UINT8   numOfNeighborBSS;
    TI_UINT8   regularBSSList[MAX_SIZE_OF_BSS_TRACK_LIST];
    TI_UINT8   numOfRegularBSS;
} listOfCandidateAps_t;

struct _roamingMngr_t
{
    /*** Roaming manager parameters that can be configured externally ***/
    roamingMngrConfig_t         	roamingMngrConfig;
    roamingMngrThresholdsConfig_t   roamingMngrThresholdsConfig;
    TI_UINT32                      	lowPassFilterRoamingAttemptInMsec;

    /*** Internal roaming parameters ***/    
    apConn_roamingTrigger_e     	roamingTrigger;				/* the roaming trigger type */
    TI_UINT8                       	currentState;				/* Roaming SM current state */    
    TI_BOOL                        	maskRoamingEvents;			/* Indicate if a trigger is already in process, and therefore the 
																	other triggers will be ignored */    
    TI_UINT32                      	lowQualityTriggerTimestamp;	/* TS to filter Too many low Quality roaming triggers */    
    scan4RoamingType_e          	scanType; 					/* the scan type performed for Roaming */    
    bssList_t                   	*pListOfAPs;				/* list of BSS received from Scan Manager */    
    TI_BOOL                        	neighborApsExist;			/* Indicating if Neighbor APs exist */    
    listOfCandidateAps_t        	listOfCandidateAps;			/* a list of the candiadte APs indexes in pListOfAPs according to
																	neighbor APs, pre-auth APs and other APs */    
    TI_UINT8                       	candidateApIndex;			/* The current candidate AP's index to Roam to */    
    TI_BOOL                        	handoverWasPerformed;		/* Indicates whether at least one handover was performed */    
    apConn_staCapabilities_t    	staCapabilities;			/* The station capabilities for the current Connection */    
    fsm_stateMachine_t          	*pRoamingSm;				/* Roaming manager SM */
                                
    /* Roaming manager handles to other objects */                                
    TI_HANDLE                   	hReport;
    TI_HANDLE                   	hOs;
    TI_HANDLE                   	hScanMngr;
    TI_HANDLE                   	hAPConnection;
    TI_HANDLE                   	hTWD;

#ifdef TI_DBG
    /* Debug trace for Roaming statistics */
    TI_UINT32                      roamingTriggerEvents[MAX_ROAMING_TRIGGERS];
    TI_UINT32                      roamingHandoverEvents[MAX_ROAMING_TRIGGERS];
    TI_UINT32                      roamingSuccesfulHandoverNum;    
    TI_UINT32                      roamingFailedHandoverNum;   
    TI_UINT32                      roamingTriggerTimestamp;
    TI_UINT32                      roamingHandoverStartedTimestamp;
    TI_UINT32                      roamingHandoverCompletedTimestamp;
    TI_UINT32                      roamingAverageSuccHandoverDuration;
    TI_UINT32                      roamingAverageRoamingDuration;
#endif
    
}; /* _roamingMngr_t */


/*****************************************************************************
**         Private Function section                                      **
*****************************************************************************/
/* SM functions */
static TI_STATUS roamingMngr_smEvent(TI_UINT8 *currState, TI_UINT8 event, void* data);
static TI_STATUS roamingMngr_smUnexpected(void *pData);
static TI_STATUS roamingMngr_smNop(void *pData);
static TI_STATUS roamingMngr_smStartIdle(void *pData);
static TI_STATUS roamingMngr_smStop(void *pData);
static TI_STATUS roamingMngr_smStopWhileScanning(void *pData);
static TI_STATUS roamingMngr_smRoamTrigger(TI_HANDLE hRoamingMngr);
static TI_STATUS roamingMngr_smInvokeScan(TI_HANDLE hRoamingMngr);
static TI_STATUS roamingMngr_smSelection(TI_HANDLE hRoamingMngr);
static TI_STATUS roamingMngr_smHandover(TI_HANDLE hRoamingMngr);
static TI_STATUS roamingMngr_smSuccHandover(TI_HANDLE hRoamingMngr);
static TI_STATUS roamingMngr_smFailHandover(TI_HANDLE hRoamingMngr);
static TI_STATUS roamingMngr_smScanFailure(TI_HANDLE hRoamingMngr);
static TI_STATUS roamingMngr_smDisconnectWhileConnecting(TI_HANDLE hRoamingMngr);


/************** callback funtions called by AP Connection **************/
/* called when a trigger for Roaming occurs */
TI_STATUS roamingMngr_triggerRoamingCb(TI_HANDLE hRoamingMngr, void *pData);
/* called when CONN status event occurs */
TI_STATUS roamingMngr_connStatusCb(TI_HANDLE hRoamingMngr, void *pData);
/* called when Neighbor APs is updated */
TI_STATUS roamingMngr_updateNeighborApListCb(TI_HANDLE hRoamingMngr, void *pData);

/* internal functions */
static void roamingMngr_releaseModule(roamingMngr_t *pRoamingMngr, TI_UINT32 initVec);

#ifdef TI_DBG
/* debug function */
static void roamingMngr_printStatistics(TI_HANDLE hRoamingMngr);
static void roamingMngr_resetStatistics(TI_HANDLE hRoamingMngr);
#endif

/**
*
* roamingMngr_releaseModule
*
* \b Description: 
*
* Called by the un load function
* Go over the vector, for each bit that is set, release the corresponding module.
*
* \b ARGS:
*
*  I   - pRoamingMngr - Roaming Manager context  \n
*  I   - initVec - indicates which modules should be released
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* \sa roamingMngr_create
*/
static void roamingMngr_releaseModule(roamingMngr_t *pRoamingMngr, TI_UINT32 initVec)
{

    if (pRoamingMngr==NULL)
    {
        return;
    }
    if (initVec & (1 << ROAMING_MNGR_SM_INIT_BIT))
    {
        fsm_Unload(pRoamingMngr->hOs, pRoamingMngr->pRoamingSm);
    }

    if (initVec & (1 << ROAMING_MNGR_CONTEXT_INIT_BIT))
    {
        os_memoryFree(pRoamingMngr->hOs, pRoamingMngr, sizeof(roamingMngr_t));
    }

    initVec = 0;
}

/**
*
* roamingMngr_triggerRoamingCb 
*
* \b Description: 
*
* This procedure is called when Roaming should be triggered
 * due to one of apConn_roamingTrigger_e Roaming Reasons.
 * Save the trigger and process it only if there's no other Roaming trigger
 * in process.
*
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*  I   - pData - pointer to roaming trigger
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
TI_STATUS roamingMngr_triggerRoamingCb(TI_HANDLE hRoamingMngr, void *pData)
{
    roamingMngr_t       *pRoamingMngr;
    apConn_roamingTrigger_e     roamingTrigger;
    TI_UINT32                      curTimestamp;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if ((pRoamingMngr == NULL) || (pData == NULL))
    {
        return TI_NOK;
    }

    roamingTrigger = *(apConn_roamingTrigger_e *)pData;

    if (roamingTrigger >= ROAMING_TRIGGER_LAST)
    {
        TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, "roamingMngr_triggerRoamingCb, bad roaming trigger = %d\n", roamingTrigger);
        return TI_NOK;
    }
#ifdef TI_DBG
    /* save parameters for debug*/
    pRoamingMngr->roamingTriggerEvents[pRoamingMngr->roamingTrigger]++;
#endif
    if (roamingTrigger <= ROAMING_TRIGGER_BG_SCAN_GROUP)
    {
        TI_BOOL    lowQuality = TI_FALSE;
        if (roamingTrigger == ROAMING_TRIGGER_LOW_QUALITY_FOR_BG_SCAN)
        {
            lowQuality = TI_TRUE;
        }
        TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_triggerRoamingCb, lowQuality = %d \n", lowQuality);
        scanMngr_qualityChangeTrigger(pRoamingMngr->hScanMngr, lowQuality);
    }
    else
    {
        if (roamingTrigger > pRoamingMngr->roamingTrigger)
        {   /* Save the highest priority roaming trigger */
            pRoamingMngr->roamingTrigger = roamingTrigger;
            TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_triggerRoamingCb, higher trigger = %d \n", roamingTrigger);

        }

        curTimestamp = os_timeStampMs(pRoamingMngr->hOs);

        /* If "No BSS" trigger received, disable count of low pass filter timer */
        if (roamingTrigger > ROAMING_TRIGGER_LOW_QUALITY_GROUP)
        {
            pRoamingMngr->lowQualityTriggerTimestamp = 0;
        }

        /* Do not invoke a new Roaming Trigger when a previous one is in process */
        if (pRoamingMngr->maskRoamingEvents == TI_FALSE)
        {   /* No Roaming trigger is in process */
            /* If the trigger is low quality check the low pass filter */
            TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_triggerRoamingCb, trigger = %d \n", roamingTrigger);
            if (roamingTrigger <= ROAMING_TRIGGER_LOW_QUALITY_GROUP)
            {
                TI_UINT32 deltaTs = curTimestamp-pRoamingMngr->lowQualityTriggerTimestamp;

                if ((pRoamingMngr->lowQualityTriggerTimestamp != 0) &&
                    (deltaTs < pRoamingMngr->lowPassFilterRoamingAttemptInMsec))
                {  /* Ignore the low quality events. till the low pass time elapses */
                    TRACE5(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_triggerRoamingCb, trigger = %d Ignored!!,deltaTs=%d, curTimestamp = %d, lowQualityTriggerTimestamp = %d, lowPassFilterRoamingAttempt=%d\n", roamingTrigger, deltaTs, curTimestamp, pRoamingMngr->lowQualityTriggerTimestamp, pRoamingMngr->lowPassFilterRoamingAttemptInMsec);
                    return TI_OK;
                }
                pRoamingMngr->lowQualityTriggerTimestamp = curTimestamp;
            }

            /* Mask all future roaming events */
            pRoamingMngr->maskRoamingEvents = TI_TRUE;

#ifdef TI_DBG
            /* For debug */
            pRoamingMngr->roamingTriggerTimestamp = curTimestamp;
#endif
            return (roamingMngr_smEvent((TI_UINT8*)&pRoamingMngr->currentState, ROAMING_EVENT_ROAM_TRIGGER, pRoamingMngr));  
        }
        else if (roamingTrigger > ROAMING_TRIGGER_FAST_CONNECT_GROUP)
        {   /* If the trigger is from the Full Connect group, then stop the connection. */
            return (roamingMngr_smEvent((TI_UINT8*)&pRoamingMngr->currentState, ROAMING_EVENT_ROAM_TRIGGER, pRoamingMngr));  
            
        }
    }

    return TI_OK;
}

/**
*
* roamingMngr_connStatusCb 
*
* \b Description: 
*
* This procedure is called when the connection status event
 * is triggered.
*
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*  I   - pData - pointer to the connection status.
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
TI_STATUS roamingMngr_connStatusCb(TI_HANDLE hRoamingMngr, void *pData)
{
    roamingMngr_t               *pRoamingMngr;
    apConn_connStatus_e         connStatus;
    roamingMngr_smEvents        roamingEvent;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if ((pRoamingMngr == NULL) || (pData == NULL))
    {
        return TI_NOK;
    }

    connStatus = ((apConn_connStatus_t *)pData)->status;
    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_connStatusCb, conn status = %d\n", connStatus);

    if (!pRoamingMngr->roamingMngrConfig.enableDisable)
    {
        TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, "roamingMngr_connStatusCb, connStatus=%d was received while Roaming is disabled. Stop Roaming \n", 						   connStatus);
        return TI_NOK;
    }

    switch (connStatus)
    {
    case CONN_STATUS_CONNECTED: roamingEvent = ROAMING_EVENT_START;
        /* Get station capabilities */
        apConn_getStaCapabilities(pRoamingMngr->hAPConnection, &pRoamingMngr->staCapabilities); 
        break;     
    case CONN_STATUS_NOT_CONNECTED: roamingEvent = ROAMING_EVENT_STOP;
        break;
    case CONN_STATUS_HANDOVER_SUCCESS: roamingEvent = ROAMING_EVENT_ROAM_SUCCESS;
#ifdef TI_DBG
        /* For debug */
        pRoamingMngr->roamingSuccesfulHandoverNum++;
        pRoamingMngr->roamingHandoverCompletedTimestamp = os_timeStampMs(pRoamingMngr->hOs);
        pRoamingMngr->roamingAverageSuccHandoverDuration += os_timeStampMs(pRoamingMngr->hOs)-pRoamingMngr->roamingHandoverStartedTimestamp;
        pRoamingMngr->roamingAverageRoamingDuration +=  os_timeStampMs(pRoamingMngr->hOs)-pRoamingMngr->roamingTriggerTimestamp;
        pRoamingMngr->roamingHandoverEvents[pRoamingMngr->roamingTrigger]++;
#endif
        break;
    case CONN_STATUS_HANDOVER_FAILURE: roamingEvent = ROAMING_EVENT_REQ_HANDOVER;
#ifdef TI_DBG
        /* For debug */
        pRoamingMngr->roamingFailedHandoverNum++;
#endif
        break;
    default:
        TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, "roamingMngr_connStatusCb, bad connStatus = %d\n", connStatus);
        return TI_NOK;
/*        break; - unreachable */
    }

    return (roamingMngr_smEvent((TI_UINT8*)&pRoamingMngr->currentState, roamingEvent, pRoamingMngr));


}

/**
*
* roamingMngr_updateNeighborApListCb 
*
* \b Description: 
*
* This procedure is called when Neighbor AP list is received from the AP.
 * Save the list, and set them in Scan Manager object.
*
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*  I   - pData - pointer to the list of Neighbor APs.
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
TI_STATUS roamingMngr_updateNeighborApListCb(TI_HANDLE hRoamingMngr, void *pData)
{
    roamingMngr_t           *pRoamingMngr;
    neighborAPList_t        *pNeighborAPList;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if ((pRoamingMngr == NULL) || (pData == NULL))
    {
        return TI_NOK;
    }

    pNeighborAPList = (neighborAPList_t *)pData;
    if (pNeighborAPList->numOfEntries>0)
    {
        pRoamingMngr->neighborApsExist = TI_TRUE;
    }
    else
    {
        pRoamingMngr->neighborApsExist = TI_FALSE;
    }
    
    if (pRoamingMngr->roamingMngrConfig.enableDisable)
    {
        scanMngr_setNeighborAPs (pRoamingMngr->hScanMngr, pNeighborAPList);
    }
    TRACE2(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_updateNeighborApListCb, numberOfAps = %d, enableDisable=%d\n", 							 pNeighborAPList->numOfEntries, pRoamingMngr->roamingMngrConfig.enableDisable);

    return TI_OK;
}

/**
*
* roamingMngr_smEvent
*
* \b Description: 
*
* Roaming Manager state machine transition function
*
* \b ARGS:
*
*  I/O - currentState - current state in the state machine\n
*  I   - event - specific event for the state machine\n
*  I   - pData - Data for state machine action function\n
*
* \b RETURNS:
*
*  TI_OK on success, TI_NOK otherwise.
*
* \sa 
*/
static TI_STATUS roamingMngr_smEvent(TI_UINT8 *currState, TI_UINT8 event, void* data)
{
    TI_STATUS       status;
    TI_UINT8           nextState;
    roamingMngr_t   *pRoamingMngr = (roamingMngr_t*)data;


    status = fsm_GetNextState(pRoamingMngr->pRoamingSm, *currState, event, &nextState);
    if (status != TI_OK)
    {
        TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, "roamingMngr_smEvent, fsm_GetNextState error\n");
        return(TI_NOK);
    }

	TRACE3(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smEvent: <currentState = %d, event = %d> --> nextState = %d\n", *currState, event, nextState);

    status = fsm_Event(pRoamingMngr->pRoamingSm, currState, event, (void *)pRoamingMngr);

#ifdef TI_DBG
    if (status != TI_OK)
    {
        TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, "roamingMngr_smEvent fsm_Event error\n");
		TRACE3(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, "roamingMngr_smEvent: <currentState = %d, event = %d> --> nextState = %d\n", *currState, event, nextState);
    }
#endif

    return status;

}

/**
*
* roamingMngr_smRoamTrigger 
*
* \b Description: 
*
* This procedure is called when an Roaming event occurs: BSS LOSS, LOW Quality etc.
 * Performs the following:
 * - If Roaming is disabled, ignore.
 * - Indicate Driver that Roaming process is starting
 * - Get the BSS list from the Scan Manager.
 * - If the list is not empty, start SELECTION
 * - If the list is empty, start SCANNING. The type of scan is decided
 *      according to the Neigbor APs existence.
*
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smRoamTrigger(TI_HANDLE hRoamingMngr)
{
    roamingMngr_t           *pRoamingMngr;
    roamingMngr_smEvents    roamingEvent;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }
    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smRoamTrigger, enableDisable = %d\n",pRoamingMngr->roamingMngrConfig.enableDisable);


    if (!pRoamingMngr->roamingMngrConfig.enableDisable)
    {   
		/* Ignore any other Roaming event when Roaming is disabled */
        TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, "roamingMngr_smRoamTrigger, when Roaming is disabled\n");
        return TI_OK;
    }
    /* Indicate the driver that Roaming process is starting */
    apConn_prepareToRoaming(pRoamingMngr->hAPConnection, pRoamingMngr->roamingTrigger);

    /* Get the current BSSIDs from ScanMngr */
    pRoamingMngr->pListOfAPs = scanMngr_getBSSList(pRoamingMngr->hScanMngr);
    if ((pRoamingMngr->pListOfAPs != NULL) && (pRoamingMngr->pListOfAPs->numOfEntries > 0))
    {   /* No need to SCAN, start SELECTING */
        roamingEvent = ROAMING_EVENT_SELECT;
    } 
    else
    {   /* check if list of APs exists in order to verify which scan to start */
        roamingEvent = ROAMING_EVENT_SCAN;
        if (pRoamingMngr->neighborApsExist)
        {   /* Scan only Neighbor APs */
            pRoamingMngr->scanType = ROAMING_PARTIAL_SCAN;
        }
        else
        {   /* Scan all channels */
            pRoamingMngr->scanType = ROAMING_FULL_SCAN;
        }
    }
    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smRoamTrigger, scanType = %d\n", pRoamingMngr->scanType);

    return (roamingMngr_smEvent((TI_UINT8*)&pRoamingMngr->currentState, roamingEvent, pRoamingMngr));
}

/**
*
* roamingMngr_smInvokeScan 
*
* \b Description: 
*
* This procedure is called when scan should be performed in order
 * to select an AP to roam to.
 * This can be the first scan, a second scan after partail scan,
 * or scan after previous scan was failed.
 * In any case, the scan can either be:
 *  partail, on list of channles or
 *  full on all channels.
*
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smInvokeScan(TI_HANDLE hRoamingMngr)
{
    roamingMngr_t       *pRoamingMngr;
    scan_mngrResultStatus_e     scanResult;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }

    /* check which scan should be performed: Partial on list of channels, or full scan */
    if ((pRoamingMngr->scanType == ROAMING_PARTIAL_SCAN) ||
        (pRoamingMngr->scanType == ROAMING_PARTIAL_SCAN_RETRY))
    {
        scanResult = scanMngr_startImmediateScan (pRoamingMngr->hScanMngr, TI_TRUE);
    }
    else
    {    /* Scan all channels */
        scanResult = scanMngr_startImmediateScan (pRoamingMngr->hScanMngr, TI_FALSE);
    }
   
    if (scanResult != SCAN_MRS_SCAN_RUNNING)
    {   /* the scan failed, immitate scan complete event */
        TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smInvokeScan, scanResult = %d\n", scanResult);
        roamingMngr_immediateScanComplete(pRoamingMngr, scanResult);
    }
    return TI_OK;

}

/**
*
* roamingMngr_smSelection 
*
* \b Description: 
*
* This procedure is called when selection should be performed.
*   It perform the following:
 * Prepare the candidate APs to roam according to:
 *  - Priority APs
 *  - Pre-Authenticated APs
 * If the candidate AP list is empty, only the current AP can be re-selected
 * Select one AP and trigger REQ_HANDOVER event.
 * 
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smSelection(TI_HANDLE hRoamingMngr)
{
    roamingMngr_t               *pRoamingMngr;
    TI_UINT32                      index;


    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }


    pRoamingMngr->listOfCandidateAps.numOfNeighborBSS = 0;
    pRoamingMngr->listOfCandidateAps.numOfPreAuthBSS = 0;
    pRoamingMngr->listOfCandidateAps.numOfRegularBSS = 0;

    pRoamingMngr->candidateApIndex = INVALID_CANDIDATE_INDEX;

    if ((pRoamingMngr->pListOfAPs == NULL) || 
        (pRoamingMngr->pListOfAPs->numOfEntries == 0))
    {   /* Error, there cannot be selection  */
        TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smSelection pListOfAPs is empty \n");
        return (roamingMngr_smEvent((TI_UINT8*)&pRoamingMngr->currentState, ROAMING_EVENT_REQ_HANDOVER, pRoamingMngr));
    }

    /* Build the candidate AP list */
    for (index=0; index<pRoamingMngr->pListOfAPs->numOfEntries; index++ )
    {
        if ( (pRoamingMngr->roamingTrigger <= ROAMING_TRIGGER_LOW_QUALITY_GROUP) &&
            (pRoamingMngr->pListOfAPs->BSSList[index].RSSI < pRoamingMngr->roamingMngrConfig.apQualityThreshold))
        {   /* Do not insert APs with low quality to the selection table, 
                if the Roaming Trigger was low Quality */
            TRACE8(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "candidate AP %x-%x-%x-%x-%x-%x with RSSI too low =%d, Quality=%d  \n", pRoamingMngr->pListOfAPs->BSSList[index].BSSID[0], pRoamingMngr->pListOfAPs->BSSList[index].BSSID[1], pRoamingMngr->pListOfAPs->BSSList[index].BSSID[2], pRoamingMngr->pListOfAPs->BSSList[index].BSSID[3], pRoamingMngr->pListOfAPs->BSSList[index].BSSID[4], pRoamingMngr->pListOfAPs->BSSList[index].BSSID[5], pRoamingMngr->pListOfAPs->BSSList[index].RSSI, pRoamingMngr->roamingMngrConfig.apQualityThreshold);

            continue;
        }

        if (apConn_isSiteBanned(pRoamingMngr->hAPConnection, &pRoamingMngr->pListOfAPs->BSSList[index].BSSID) == TI_TRUE)
        {
            TRACE6(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, ": Candidate AP %02X-%02X-%02X-%02X-%02X-%02X is banned!\n",									 pRoamingMngr->pListOfAPs->BSSList[index].BSSID[0],	pRoamingMngr->pListOfAPs->BSSList[index].BSSID[1], pRoamingMngr->pListOfAPs->BSSList[index].BSSID[2], pRoamingMngr->pListOfAPs->BSSList[index].BSSID[3], pRoamingMngr->pListOfAPs->BSSList[index].BSSID[4], pRoamingMngr->pListOfAPs->BSSList[index].BSSID[5]);
            continue;
        }

        if (pRoamingMngr->pListOfAPs->BSSList[index].bNeighborAP)
        {   /* The AP is a neighbor AP, insert its index to the neighbor APs list */
            pRoamingMngr->listOfCandidateAps.neighborBSSList[pRoamingMngr->listOfCandidateAps.numOfNeighborBSS] = index; 
            pRoamingMngr->listOfCandidateAps.numOfNeighborBSS++;
        }
        else if (apConn_getPreAuthAPStatus(pRoamingMngr->hAPConnection, 
										   &pRoamingMngr->pListOfAPs->BSSList[index].BSSID))
        {   /* This AP is a pre-auth AP */
            pRoamingMngr->listOfCandidateAps.preAuthBSSList[pRoamingMngr->listOfCandidateAps.numOfPreAuthBSS] = index; 
            pRoamingMngr->listOfCandidateAps.numOfPreAuthBSS++;
        }
        else
        {   /* This AP is not Neighbor nor Pre-Auth */
            pRoamingMngr->listOfCandidateAps.regularBSSList[pRoamingMngr->listOfCandidateAps.numOfRegularBSS] = index; 
            pRoamingMngr->listOfCandidateAps.numOfRegularBSS++;
        }
    }

#ifdef TI_DBG
    {   /* for debug */
        paramInfo_t     param;

        param.paramType = ROAMING_MNGR_PRINT_CANDIDATE_TABLE;
        roamingMngr_getParam(pRoamingMngr, &param);

    }
#endif
    return (roamingMngr_smEvent((TI_UINT8*)&pRoamingMngr->currentState, ROAMING_EVENT_REQ_HANDOVER, pRoamingMngr));

}

/**
*
* roamingMngr_smHandover 
*
* \b Description: 
*
* This procedure is called when handover should be invoked.
*   Go over the candidate APs and start handover to each of them. 
 * If there's no candidate APs, disconnect.
 * Handover to the current AP is allowed only if the trigger is
 * low quality.
 * 
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smHandover(TI_HANDLE hRoamingMngr)
{
    roamingMngr_t           *pRoamingMngr;
    bssEntry_t              *pApToConnect;
    apConn_connRequest_t    requestToApConn;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }


    if ((pRoamingMngr->handoverWasPerformed) && (pRoamingMngr->candidateApIndex == CURRENT_AP_INDEX))
    {   /* Handover with the current AP already failed, Disconnect */
        return (roamingMngr_smEvent((TI_UINT8*)&pRoamingMngr->currentState, ROAMING_EVENT_FAILURE, pRoamingMngr));
    }
    if (pRoamingMngr->listOfCandidateAps.numOfNeighborBSS > 0)
    {   /* Neighbor APs are the highest priority to Roam */
        pRoamingMngr->candidateApIndex = 
            pRoamingMngr->listOfCandidateAps.neighborBSSList[pRoamingMngr->listOfCandidateAps.numOfNeighborBSS-1];
        pRoamingMngr->listOfCandidateAps.numOfNeighborBSS--;
    }
    else if (pRoamingMngr->listOfCandidateAps.numOfPreAuthBSS > 0)
    {   /* Pre-Auth APs are the second priority to Roam */
        pRoamingMngr->candidateApIndex = 
            pRoamingMngr->listOfCandidateAps.preAuthBSSList[pRoamingMngr->listOfCandidateAps.numOfPreAuthBSS-1];
        pRoamingMngr->listOfCandidateAps.numOfPreAuthBSS--;
    }
    else if (pRoamingMngr->listOfCandidateAps.numOfRegularBSS > 0)
    {   /* Regular APs are APs that are not pre-authenticated and not Neighbor */
        pRoamingMngr->candidateApIndex = 
            pRoamingMngr->listOfCandidateAps.regularBSSList[pRoamingMngr->listOfCandidateAps.numOfRegularBSS-1];
        pRoamingMngr->listOfCandidateAps.numOfRegularBSS--;
    }
    else
    {   /* No Candidate APs */
        pRoamingMngr->candidateApIndex = INVALID_CANDIDATE_INDEX;
    }

    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smHandover, candidateApIndex=%d \n", pRoamingMngr->candidateApIndex);


    if (pRoamingMngr->candidateApIndex == INVALID_CANDIDATE_INDEX)
    {   /* No cnadidate to Roam to, only the current AP is candidate */
        if (pRoamingMngr->roamingTrigger <= ROAMING_TRIGGER_LOW_QUALITY_GROUP)
        {   /* If the trigger to Roam is low quality, and there are no candidate APs
                to roam to, retain connected to the current AP */
            requestToApConn.requestType = (pRoamingMngr->handoverWasPerformed) ? AP_CONNECT_RECONNECT_CURR_AP : AP_CONNECT_RETAIN_CURR_AP;
            pRoamingMngr->candidateApIndex = CURRENT_AP_INDEX;
        }
        else
        {   /* Disconnect the BSS, there are no more APs to roam to */
            return (roamingMngr_smEvent((TI_UINT8*)&pRoamingMngr->currentState, ROAMING_EVENT_FAILURE, pRoamingMngr));
        }
    }
    else
    {   /* There is a valid candidate AP */
        if (pRoamingMngr->roamingTrigger > ROAMING_TRIGGER_FAST_CONNECT_GROUP)
        {   /* Full re-connection should be perfromed */
            requestToApConn.requestType = AP_CONNECT_FULL_TO_AP; 
        }
        else
        {   /* Fast re-connection should be perfromed */
            requestToApConn.requestType = AP_CONNECT_FAST_TO_AP; 
        }
    }
#ifdef TI_DBG
    /* For debug */
    if (!pRoamingMngr->handoverWasPerformed)
    {   /* Take the time before the first handover started */
        pRoamingMngr->roamingHandoverStartedTimestamp = os_timeStampMs(pRoamingMngr->hOs);
    }
#endif
    
    if (pRoamingMngr->candidateApIndex == CURRENT_AP_INDEX)
    {   /* get the current AP */
        pApToConnect = apConn_getBSSParams(pRoamingMngr->hAPConnection);
    }
    else
    {   /* get the candidate AP */
        pRoamingMngr->handoverWasPerformed = TI_TRUE;
        pApToConnect = &pRoamingMngr->pListOfAPs->BSSList[pRoamingMngr->candidateApIndex];
    }
    TRACE3(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smHandover, candidateApIndex=%d, requestType = %d, channel=%d \n", 							 pRoamingMngr->candidateApIndex, requestToApConn.requestType, pApToConnect->channel);

    requestToApConn.dataBufLength = 0;
    return (apConn_connectToAP(pRoamingMngr->hAPConnection, pApToConnect, &requestToApConn, TI_TRUE));
}

/**
*
* roamingMngr_smDisconnectWhileConnecting 
*
* \b Description: 
*
* This procedure is called when the Station is in the process of connection,
 * and the AP disconnects the station. 
 * 
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smDisconnectWhileConnecting(TI_HANDLE hRoamingMngr)
{
    roamingMngr_t           *pRoamingMngr;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }

    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smDisconnectWhileConnecting, candidateApIndex=%d \n", pRoamingMngr->candidateApIndex);

    if (pRoamingMngr->roamingTrigger > ROAMING_TRIGGER_FAST_CONNECT_GROUP)
    {   /* If the trigger is from the Full Connect group, then stop the connection. */
        /* clean intenal variables */
        pRoamingMngr->maskRoamingEvents = TI_TRUE;
        pRoamingMngr->roamingTrigger = ROAMING_TRIGGER_NONE;

        scanMngr_stopContScan(pRoamingMngr->hScanMngr);
#ifdef TI_DBG
        pRoamingMngr->roamingFailedHandoverNum++;
#endif
        return (apConn_disconnect(pRoamingMngr->hAPConnection));
        
    }

    return TI_OK;

}

/**
*
* roamingMngr_smSuccHandover 
*
* \b Description: 
*
* This procedure is called when handover succeeded.
 * Inform Scan Manager about the new AP.    
 * UnMask Roaming Triggers. 
 * 
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smSuccHandover(TI_HANDLE hRoamingMngr)
{
    roamingMngr_t           *pRoamingMngr;
    bssEntry_t              *pNewConnectedAp;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }

    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smSuccHandover, candidateApIndex=%d \n", pRoamingMngr->candidateApIndex);

    if (pRoamingMngr->handoverWasPerformed &&
        (pRoamingMngr->pListOfAPs != NULL) &&
        (pRoamingMngr->pListOfAPs->numOfEntries>0))
    {
        if (pRoamingMngr->candidateApIndex == CURRENT_AP_INDEX)
        {   
			/* get the current AP */
            pNewConnectedAp = apConn_getBSSParams(pRoamingMngr->hAPConnection);
        }
        else
        {   
			/* get the candidate AP */
            pNewConnectedAp = &pRoamingMngr->pListOfAPs->BSSList[pRoamingMngr->candidateApIndex];
        }

        scanMngr_handoverDone(pRoamingMngr->hScanMngr, 
							  &pNewConnectedAp->BSSID,
							  pNewConnectedAp->band);
    }
    pRoamingMngr->maskRoamingEvents = TI_FALSE;
    pRoamingMngr->candidateApIndex = INVALID_CANDIDATE_INDEX;
    pRoamingMngr->handoverWasPerformed = TI_FALSE;
    pRoamingMngr->roamingTrigger = ROAMING_TRIGGER_NONE;

    /* Start pre-authentication in order to set PMKID
        for the current AP */
    if (pRoamingMngr->staCapabilities.authMode==os802_11AuthModeWPA2)
    {   
		/* No Pre-Auth is required */
        bssList_t           bssList;

        TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smStartIdle, Pre-Auth to cur AP\n");
        bssList.numOfEntries = 0;
        apConn_preAuthenticate(pRoamingMngr->hAPConnection, &bssList);
    }

    return TI_OK;
}

/**
*
* roamingMngr_smFailHandover 
*
* \b Description: 
*
* This procedure is called when handover failed and there are no more
 * APs to roam to. Disconnect the BSS and retrun to IDLE state.
* 
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smFailHandover(TI_HANDLE hRoamingMngr)
{
    roamingMngr_t           *pRoamingMngr;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }

    TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smFailHandover \n");

    /* clean intenal variables */
    pRoamingMngr->maskRoamingEvents = TI_TRUE;
    pRoamingMngr->roamingTrigger = ROAMING_TRIGGER_NONE;

    scanMngr_stopContScan(pRoamingMngr->hScanMngr);
#ifdef TI_DBG
    pRoamingMngr->roamingFailedHandoverNum++;
#endif
    return (apConn_disconnect(pRoamingMngr->hAPConnection));
}

/**
*
* roamingMngr_smScanFailure 
*
* \b Description: 
*
* This procedure is called when all scan attempts failed. 
 * Send Disconnect event and return to IDLE state.
 *
* 
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smScanFailure(TI_HANDLE hRoamingMngr)
{
    roamingMngr_t           *pRoamingMngr;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }

    TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smScanFailure \n");

    /* clean intenal variables */
    pRoamingMngr->maskRoamingEvents = TI_TRUE;
    pRoamingMngr->roamingTrigger = ROAMING_TRIGGER_NONE;

    scanMngr_stopContScan(pRoamingMngr->hScanMngr);

    return (apConn_disconnect(pRoamingMngr->hAPConnection));
}

#if 0
/**
*
* roamingMngr_smCmdFailure 
*
* \b Description: 
*
* This procedure is called when all the driver failed to prepare to Roaming. 
 * Mask all future Roaming triggers.
 *
* 
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smCmdFailure(TI_HANDLE hRoamingMngr)
{
    roamingMngr_t           *pRoamingMngr;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }

    TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smCmdFailure \n");

    /* clean intenal variables */
    pRoamingMngr->maskRoamingEvents = TI_TRUE;
    pRoamingMngr->roamingTrigger = ROAMING_TRIGGER_NONE;

    return TI_OK;
    
}
#endif

/**
*
* roamingMngr_smStartIdle - Start event when in Idle state
*
* \b Description: 
*
* Start event when in Idle state. 
 * This function is called when the station becomes CONNECTED.
 * Perform the following:
 * - The current state becomes WAIT_4_TRIGGER 
 * - Unmask Roaming events
 * - Set handoverWasPerformed to TI_FALSE
 * - Start the Scan Manager
*
* \b ARGS:
*
*  I   - pData - pointer to the roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smStartIdle(void *pData)
{
    roamingMngr_t       *pRoamingMngr;
    bssEntry_t          *pCurBssEntry;

    pRoamingMngr = (roamingMngr_t*)pData;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }
    TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smStartIdle, Unmask Roaming events and start continuos scan \n");

    pRoamingMngr->maskRoamingEvents = TI_FALSE;
    pRoamingMngr->handoverWasPerformed = TI_FALSE;
    pRoamingMngr->roamingTrigger = ROAMING_TRIGGER_NONE;

    pCurBssEntry = apConn_getBSSParams(pRoamingMngr->hAPConnection);
    scanMngr_startContScan(pRoamingMngr->hScanMngr, &pCurBssEntry->BSSID, pCurBssEntry->band);

    /* Start pre-authentication in order to set PMKID
        for the current AP */
    if (pRoamingMngr->staCapabilities.authMode==os802_11AuthModeWPA2)
    {   /* No Pre-Auth is required */
        bssList_t           bssList;

        TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_smStartIdle, Pre-Auth to cur AP\n");
        bssList.numOfEntries = 0;
        apConn_preAuthenticate(pRoamingMngr->hAPConnection, &bssList);
    }

    return TI_OK;
}

/**
*
* roamingMngr_smNop - Do nothing
*
* \b Description: 
*
* Do nothing in the SM.
*
* \b ARGS:
*
*  I   - pData - pointer to the roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smNop(void *pData)
{
    roamingMngr_t       *pRoamingMngr;

    pRoamingMngr = (roamingMngr_t*)pData;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }
    TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, " roamingMngr_smNop\n");
    
    return TI_OK;
}

/**
*
* roamingMngr_smUnexpected - Unexpected event
*
* \b Description: 
*
* Unexpected event in the SM.
*
* \b ARGS:
*
*  I   - pData - pointer to the roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smUnexpected(void *pData)
{
    roamingMngr_t       *pRoamingMngr;

    pRoamingMngr = (roamingMngr_t*)pData;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }
    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, " roamingMngr_smUnexpected, state = %d\n", pRoamingMngr->currentState);
    
    return TI_NOK;
}

/**
*
* roamingMngr_smStop - Stop all timers and clean DB
*
* \b Description: 
*
* Stop event in start state. Stop timers, clean internal vars
 *  and exit PS if necessary.
*
* \b ARGS:
*
*  I   - pData - pointer to the roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smStop(void *pData)
{
    roamingMngr_t       *pRoamingMngr;

    pRoamingMngr = (roamingMngr_t*)pData;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }
    TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, " roamingMngr_smStop\n");

    scanMngr_stopContScan(pRoamingMngr->hScanMngr);
    /* clean intenal variables */
    pRoamingMngr->maskRoamingEvents = TI_TRUE;
    pRoamingMngr->neighborApsExist = TI_FALSE;
    pRoamingMngr->roamingTrigger = ROAMING_TRIGGER_NONE;

    return TI_OK;
}
/**
*
* roamingMngr_smStopWhileScanning - 
*
* \b Description: 
*
* Stop event means that the station is not in Connected State. 
 * Stop continuos and immediate scans and clean internal vars.
*
* \b ARGS:
*
*  I   - pData - pointer to the roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static TI_STATUS roamingMngr_smStopWhileScanning(void *pData)
{
    roamingMngr_t       *pRoamingMngr;

    pRoamingMngr = (roamingMngr_t*)pData;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }
    TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, " roamingMngr_smStopWhileScanning\n");

    scanMngr_stopImmediateScan(pRoamingMngr->hScanMngr);
    scanMngr_stopContScan(pRoamingMngr->hScanMngr);

    /* clean intenal variables */
    pRoamingMngr->maskRoamingEvents = TI_TRUE;
    pRoamingMngr->neighborApsExist = TI_FALSE;
    pRoamingMngr->roamingTrigger = ROAMING_TRIGGER_NONE;

    return TI_OK;
}

  
#ifdef TI_DBG
/**
*
* roamingMngr_debugTrace 
*
* \b Description: 
*
* This procedure is called for debug only, to trace the roaming triggers and events
*
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static void roamingMngr_printStatistics(TI_HANDLE hRoamingMngr)
{


    roamingMngr_t       *pRoamingMngr;
    TI_UINT8               index;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return;
    }

    WLAN_OS_REPORT(("******** ROAMING_TRIGGERS ********\n"));
    for (index=ROAMING_TRIGGER_LOW_TX_RATE; index<ROAMING_TRIGGER_LAST; index++)
    {
        switch (index)
        {
        case ROAMING_TRIGGER_LOW_TX_RATE:
            WLAN_OS_REPORT(("- Low TX rate = %d\n",     pRoamingMngr->roamingTriggerEvents[index]));
            break;
        case ROAMING_TRIGGER_LOW_SNR:
            WLAN_OS_REPORT(("- Low Snr = %d\n",         pRoamingMngr->roamingTriggerEvents[index]));
            break;
        case ROAMING_TRIGGER_LOW_QUALITY:
            WLAN_OS_REPORT(("- Low Quality = %d\n",     pRoamingMngr->roamingTriggerEvents[index]));
            break;
        case ROAMING_TRIGGER_MAX_TX_RETRIES:
            WLAN_OS_REPORT(("- MAX TX retries = %d\n",  pRoamingMngr->roamingTriggerEvents[index]));
            break;
        case ROAMING_TRIGGER_BSS_LOSS:
            WLAN_OS_REPORT(("- BSS Loss TX = %d\n",     pRoamingMngr->roamingTriggerEvents[index]));
            break;
        case ROAMING_TRIGGER_SWITCH_CHANNEL:
            WLAN_OS_REPORT(("- Switch Channel = %d\n",  pRoamingMngr->roamingTriggerEvents[index]));
            break;
        case ROAMING_TRIGGER_AP_DISCONNECT:
            WLAN_OS_REPORT(("- AP Disconnect = %d\n",   pRoamingMngr->roamingTriggerEvents[index]));
            break;
        case ROAMING_TRIGGER_SECURITY_ATTACK:
            WLAN_OS_REPORT(("- SEC attack = %d\n",      pRoamingMngr->roamingTriggerEvents[index]));
            break;  
        default:
            break;
        }
    }

    WLAN_OS_REPORT(("******** Succ ROAMING_HANDOVERS ********\n"));

    for (index=ROAMING_TRIGGER_LOW_QUALITY; index<ROAMING_TRIGGER_LAST; index++)
    {
        switch (index)
        {
        case ROAMING_TRIGGER_LOW_TX_RATE:               
            WLAN_OS_REPORT(("- Low TX rate = %d\n",     pRoamingMngr->roamingHandoverEvents[index]));
            break;                                     
        case ROAMING_TRIGGER_LOW_SNR:               
            WLAN_OS_REPORT(("- Low Snre = %d\n",        pRoamingMngr->roamingHandoverEvents[index]));
            break;                                     
        case ROAMING_TRIGGER_LOW_QUALITY:               
            WLAN_OS_REPORT(("- Low Quality = %d\n",     pRoamingMngr->roamingHandoverEvents[index]));
            break;                                     
        case ROAMING_TRIGGER_MAX_TX_RETRIES:            
            WLAN_OS_REPORT(("- MAX TX retries = %d\n",  pRoamingMngr->roamingHandoverEvents[index]));
            break;                                     
        case ROAMING_TRIGGER_BSS_LOSS:                  
            WLAN_OS_REPORT(("- BSS Loss TX = %d\n",     pRoamingMngr->roamingHandoverEvents[index]));
            break;                                     
        case ROAMING_TRIGGER_SWITCH_CHANNEL:            
            WLAN_OS_REPORT(("- Switch Channel = %d\n",   pRoamingMngr->roamingHandoverEvents[index]));
            break;                                     
        case ROAMING_TRIGGER_AP_DISCONNECT:             
            WLAN_OS_REPORT(("- AP Disconnect = %d\n",   pRoamingMngr->roamingHandoverEvents[index]));
            break;                                     
        case ROAMING_TRIGGER_SECURITY_ATTACK:           
            WLAN_OS_REPORT(("- SEC attack = %d\n",      pRoamingMngr->roamingHandoverEvents[index])); 
            break;                                     
        default:
            break;
        }
    }

    WLAN_OS_REPORT(("******** ROAMING STATISTICS ********\n"));
    WLAN_OS_REPORT(("- Num of succesful handovers = %d\n", pRoamingMngr->roamingSuccesfulHandoverNum)); 
    WLAN_OS_REPORT(("- Num of failed handovers = %d\n", pRoamingMngr->roamingFailedHandoverNum)); 
    if (pRoamingMngr->roamingSuccesfulHandoverNum >0)
    {
        WLAN_OS_REPORT(("- Succesful average succesful handover duration = %d\n", pRoamingMngr->roamingAverageSuccHandoverDuration/pRoamingMngr->roamingSuccesfulHandoverNum)); 
        WLAN_OS_REPORT(("- Succesful average roaming duration = %d\n", pRoamingMngr->roamingAverageRoamingDuration/pRoamingMngr->roamingSuccesfulHandoverNum)); 
    }


}


/**
*
* roamingMngr_resetDebugTrace 
*
* \b Description: 
*
* This procedure is called for debug only, to reset Roaming debug trace 
*
* \b ARGS:
*
*  I   - hRoamingMngr - roamingMngr SM context  \n
*
* \b RETURNS:
*
*  TI_OK if successful, TI_NOK otherwise.
*
* 
*/
static void roamingMngr_resetStatistics(TI_HANDLE hRoamingMngr)
{

    roamingMngr_t       *pRoamingMngr;
    TI_UINT8               index;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return;
    }
    WLAN_OS_REPORT(("Resetting all ROAMING_EVENTS \n"));

    pRoamingMngr->roamingSuccesfulHandoverNum = 0;    
    pRoamingMngr->roamingHandoverStartedTimestamp = 0;  
    pRoamingMngr->roamingHandoverCompletedTimestamp = 0;
    pRoamingMngr->roamingAverageSuccHandoverDuration = 0; 
    pRoamingMngr->roamingAverageRoamingDuration = 0;  
    pRoamingMngr->roamingFailedHandoverNum = 0;

    for (index=ROAMING_TRIGGER_LOW_QUALITY; index<ROAMING_TRIGGER_LAST; index++)
    {
        pRoamingMngr->roamingHandoverEvents[index] = 0;
        pRoamingMngr->roamingTriggerEvents[index] = 0;
    }
}

#endif /*TI_DBG*/



/**********************************************************************
**         External Function section                                 **
***********************************************************************/
extern TI_STATUS apConn_reportRoamingEvent(TI_HANDLE hAPConnection,
										   apConn_roamingTrigger_e roamingEventType,
										   void *roamingEventData);



/**********************************************************************
**         API Function section                                      **
***********************************************************************/

TI_HANDLE roamingMngr_create(TI_HANDLE hOs)
{
    TI_STATUS       status;
    roamingMngr_t   *pRoamingMngr;
    TI_UINT32          initVec;

    initVec = 0;

    pRoamingMngr = os_memoryAlloc(hOs, sizeof(roamingMngr_t));
    if (pRoamingMngr == NULL)
        return NULL;

    initVec |= (1 << ROAMING_MNGR_CONTEXT_INIT_BIT);
    pRoamingMngr->hOs   = hOs;

    status = fsm_Create(hOs, &pRoamingMngr->pRoamingSm, ROAMING_MNGR_NUM_STATES, ROAMING_MNGR_NUM_EVENTS);
    if (status != TI_OK)
    {
        roamingMngr_releaseModule(pRoamingMngr, initVec);
        WLAN_OS_REPORT(("FATAL ERROR: roamingMngr_create(): Error Creating pRoamingSm - Aborting\n"));
        return NULL;
    }
    initVec |= (1 << ROAMING_MNGR_SM_INIT_BIT);

    
    return pRoamingMngr;
}

TI_STATUS roamingMngr_unload(TI_HANDLE hRoamingMngr)
{
    TI_UINT32          initVec;

    if (hRoamingMngr == NULL)
    {
        return TI_OK;
    }

    initVec = 0xFFFF;
    roamingMngr_releaseModule(hRoamingMngr, initVec);

    return TI_OK;
}

void roamingMngr_init (TStadHandlesList *pStadHandles)
{
    roamingMngr_t  *pRoamingMngr = (roamingMngr_t*)(pStadHandles->hRoamingMngr);
#ifdef TI_DBG
    TI_UINT8        index;
#endif


    /** Station broadcast key State Machine matrix */
    fsm_actionCell_t    roamingMngr_matrix[ROAMING_MNGR_NUM_STATES][ROAMING_MNGR_NUM_EVENTS] =
    {
        /* next state and actions for IDLE state */
        {   {ROAMING_STATE_WAIT_4_TRIGGER, roamingMngr_smStartIdle},        /* START            */
            {ROAMING_STATE_IDLE, roamingMngr_smNop},                        /* STOP             */ 
            {ROAMING_STATE_IDLE, roamingMngr_smNop},                        /* ROAM_TRIGGER     */
            {ROAMING_STATE_IDLE, roamingMngr_smUnexpected},                 /* SCAN             */
            {ROAMING_STATE_IDLE, roamingMngr_smUnexpected},                 /* SELECT           */
            {ROAMING_STATE_IDLE, roamingMngr_smUnexpected},                 /* REQ_HANDOVER     */
            {ROAMING_STATE_IDLE, roamingMngr_smUnexpected},                 /* ROAM_SUCCESS     */
            {ROAMING_STATE_IDLE, roamingMngr_smUnexpected}                  /* FAILURE          */
        },                                                                              

        /* next state and actions for WAIT_4_TRIGGER state */    
        {   {ROAMING_STATE_WAIT_4_TRIGGER, roamingMngr_smUnexpected},   /* START            */ 
            {ROAMING_STATE_IDLE, roamingMngr_smStop},                   /* STOP             */ 
            {ROAMING_STATE_WAIT_4_CMD, roamingMngr_smRoamTrigger},      /* ROAM_TRIGGER     */ 
            {ROAMING_STATE_WAIT_4_TRIGGER, roamingMngr_smUnexpected},   /* SCAN             */ 
            {ROAMING_STATE_WAIT_4_TRIGGER, roamingMngr_smUnexpected},   /* SELECT           */ 
            {ROAMING_STATE_WAIT_4_TRIGGER, roamingMngr_smUnexpected},   /* REQ_HANDOVER     */ 
            {ROAMING_STATE_WAIT_4_TRIGGER, roamingMngr_smUnexpected},   /* ROAM_SUCCESS     */ 
            {ROAMING_STATE_WAIT_4_TRIGGER, roamingMngr_smUnexpected}    /* FAILURE          */
        },                                                                              
    
        /* next state and actions for WAIT_4_CMD state */    
        {   {ROAMING_STATE_WAIT_4_CMD, roamingMngr_smUnexpected},               /* START            */ 
            {ROAMING_STATE_WAIT_4_CMD, roamingMngr_smUnexpected},               /* STOP             */ 
            {ROAMING_STATE_WAIT_4_CMD, roamingMngr_smUnexpected},               /* ROAM_TRIGGER     */ 
            {ROAMING_STATE_SCANNING, roamingMngr_smInvokeScan},                 /* SCAN             */ 
            {ROAMING_STATE_SELECTING, roamingMngr_smSelection},                 /* SELECT           */ 
            {ROAMING_STATE_WAIT_4_CMD, roamingMngr_smUnexpected},               /* REQ_HANDOVER     */ 
            {ROAMING_STATE_WAIT_4_CMD, roamingMngr_smUnexpected},               /* ROAM_SUCCESS     */ 
            {ROAMING_STATE_WAIT_4_CMD, roamingMngr_smUnexpected}                /* FAILURE          */ 
        },                                                                              

        /* next state and actions for SCANNING state */                      
        {   {ROAMING_STATE_SCANNING, roamingMngr_smUnexpected},              /* START            */
            {ROAMING_STATE_IDLE, roamingMngr_smStopWhileScanning},           /* STOP             */
            {ROAMING_STATE_SCANNING, roamingMngr_smNop},                     /* ROAM_TRIGGER     */
            {ROAMING_STATE_SCANNING, roamingMngr_smInvokeScan},              /* SCAN             */
            {ROAMING_STATE_SELECTING, roamingMngr_smSelection},              /* SELECT           */
            {ROAMING_STATE_SCANNING, roamingMngr_smUnexpected},              /* REQ_HANDOVER     */
            {ROAMING_STATE_SCANNING, roamingMngr_smUnexpected},              /* ROAM_SUCCESS     */
            {ROAMING_STATE_IDLE, roamingMngr_smScanFailure}                  /* FAILURE          */
                                                                                   
        }, 

        /* next state and actions for SELECTING state */                      
        {   {ROAMING_STATE_SELECTING, roamingMngr_smUnexpected},             /* START            */
            {ROAMING_STATE_SELECTING, roamingMngr_smUnexpected},             /* STOP             */
            {ROAMING_STATE_SELECTING, roamingMngr_smUnexpected},             /* ROAM_TRIGGER     */
            {ROAMING_STATE_SELECTING, roamingMngr_smUnexpected},             /* SCAN             */
            {ROAMING_STATE_SELECTING, roamingMngr_smUnexpected},             /* SELECT           */
            {ROAMING_STATE_CONNECTING, roamingMngr_smHandover},              /* REQ_HANDOVER     */
            {ROAMING_STATE_SELECTING, roamingMngr_smUnexpected},             /* ROAM_SUCCESS     */
            {ROAMING_STATE_SELECTING, roamingMngr_smUnexpected}              /* FAILURE          */
                                                                                   
        }, 

        /* next state and actions for CONNECTING state */                      
        {   {ROAMING_STATE_CONNECTING, roamingMngr_smUnexpected},           /* START            */ 
            {ROAMING_STATE_IDLE, roamingMngr_smStop},                       /* STOP             */ 
            {ROAMING_STATE_IDLE, roamingMngr_smDisconnectWhileConnecting},      /* ROAM_TRIGGER     */ 
            {ROAMING_STATE_CONNECTING, roamingMngr_smUnexpected},           /* SCAN,            */ 
            {ROAMING_STATE_CONNECTING, roamingMngr_smUnexpected},           /* SELECT           */ 
            {ROAMING_STATE_CONNECTING, roamingMngr_smHandover},             /* REQ_HANDOVER     */ 
            {ROAMING_STATE_WAIT_4_TRIGGER, roamingMngr_smSuccHandover} ,    /* ROAM_SUCCESS     */ 
            {ROAMING_STATE_IDLE, roamingMngr_smFailHandover}                /* FAILURE          */ 
                                                                                   
        } 
    }; 

    /* Update handlers */
    pRoamingMngr->hReport       = pStadHandles->hReport;
    pRoamingMngr->hScanMngr     = pStadHandles->hScanMngr;
    pRoamingMngr->hAPConnection = pStadHandles->hAPConnection;
    pRoamingMngr->hTWD          = pStadHandles->hTWD;

    /* Init intrenal variables */
    pRoamingMngr->currentState = ROAMING_STATE_IDLE;
    pRoamingMngr->roamingMngrConfig.enableDisable = ROAMING_DISABLED; 
    pRoamingMngr->roamingMngrConfig.apQualityThreshold = DEFAULT_AP_QUALITY;
    pRoamingMngr->roamingMngrConfig.lowPassFilterRoamingAttempt = DEFAULT_LOW_PASS_FILTER;
    pRoamingMngr->roamingTrigger = ROAMING_TRIGGER_NONE;
    pRoamingMngr->maskRoamingEvents= TI_TRUE;
    pRoamingMngr->scanType = ROAMING_NO_SCAN;
    pRoamingMngr->candidateApIndex = INVALID_CANDIDATE_INDEX;
    pRoamingMngr->handoverWasPerformed = TI_FALSE;
    pRoamingMngr->lowQualityTriggerTimestamp = 0;
    pRoamingMngr->neighborApsExist = TI_FALSE;
    pRoamingMngr->pListOfAPs = NULL;
    pRoamingMngr->candidateApIndex = INVALID_CANDIDATE_INDEX;
    pRoamingMngr->listOfCandidateAps.numOfNeighborBSS = 0;
    pRoamingMngr->listOfCandidateAps.numOfPreAuthBSS = 0;
    pRoamingMngr->listOfCandidateAps.numOfRegularBSS = 0;

#ifdef TI_DBG
    /* debug counters */
    pRoamingMngr->roamingSuccesfulHandoverNum = 0;    
    pRoamingMngr->roamingHandoverStartedTimestamp = 0;  
    pRoamingMngr->roamingHandoverCompletedTimestamp = 0;
    pRoamingMngr->roamingAverageSuccHandoverDuration = 0; 
    pRoamingMngr->roamingAverageRoamingDuration = 0;  
    pRoamingMngr->roamingFailedHandoverNum = 0;

    for (index=ROAMING_TRIGGER_NONE; index<ROAMING_TRIGGER_LAST; index++)
    {
        pRoamingMngr->roamingTriggerEvents[index] = 0;
        pRoamingMngr->roamingHandoverEvents[index] = 0;
    }
#endif

    /* config the FSM */
    fsm_Config(pRoamingMngr->pRoamingSm, 
               &roamingMngr_matrix[0][0], 
               ROAMING_MNGR_NUM_STATES, 
               ROAMING_MNGR_NUM_EVENTS, 
               roamingMngr_smEvent, pRoamingMngr->hOs);
}


TI_STATUS roamingMngr_setParam(TI_HANDLE hRoamingMngr, paramInfo_t *pParam)
{
    roamingMngr_t *pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    TI_STATUS      status       = TI_OK;

    if (pParam == NULL)
    {
        TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR , "roamingMngr_setParam(): pParam is NULL!\n");
        return TI_NOK;
    }

    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION , "roamingMngr_setParam   %X \n", pParam->paramType);

    switch (pParam->paramType)
    {
    
    case ROAMING_MNGR_APPLICATION_CONFIGURATION:
        {
            roamingMngrConfigParams_t   *pRoamingMngrConfigParams;
    
            pRoamingMngrConfigParams = &pParam->content.roamingConfigBuffer;
    
            /* Configure the Roaming Parmeters */
            TRACE3(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_setParam Configuration: \n                                  enableDisable= %d,\n  lowPassFilterRoamingAttempt=%d,\n                                  apQualityThreshold=%d\n", pRoamingMngrConfigParams->roamingMngrConfig.enableDisable, pRoamingMngrConfigParams->roamingMngrConfig.lowPassFilterRoamingAttempt, pRoamingMngrConfigParams->roamingMngrConfig.apQualityThreshold);

            pRoamingMngr->roamingMngrConfig.apQualityThreshold = pRoamingMngrConfigParams->roamingMngrConfig.apQualityThreshold;
            pRoamingMngr->roamingMngrConfig.lowPassFilterRoamingAttempt = pRoamingMngrConfigParams->roamingMngrConfig.lowPassFilterRoamingAttempt;
            pRoamingMngr->lowPassFilterRoamingAttemptInMsec = pRoamingMngrConfigParams->roamingMngrConfig.lowPassFilterRoamingAttempt * 1000;

            /* Configure the Roaming Trigger thresholds */
            TRACE7(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_setParam Thresholds: \n                                  dataRetryThreshold= %d,\n  lowQualityForBackgroungScanCondition=%d,\n                                  lowRssiThreshold=%d,\n lowSNRThreshold=%d,\n                                  normalQualityForBackgroungScanCondition=%d,\n                                  numExpectedTbttForBSSLoss=%d,\n txRateThreshold=%d \n \n", pRoamingMngrConfigParams->roamingMngrThresholdsConfig.dataRetryThreshold, pRoamingMngrConfigParams->roamingMngrThresholdsConfig.lowQualityForBackgroungScanCondition, pRoamingMngrConfigParams->roamingMngrThresholdsConfig.lowRssiThreshold, pRoamingMngrConfigParams->roamingMngrThresholdsConfig.lowSnrThreshold, pRoamingMngrConfigParams->roamingMngrThresholdsConfig.normalQualityForBackgroungScanCondition, pRoamingMngrConfigParams->roamingMngrThresholdsConfig.numExpectedTbttForBSSLoss, pRoamingMngrConfigParams->roamingMngrThresholdsConfig.txRateThreshold);

            os_memoryCopy(pRoamingMngr->hOs, &pRoamingMngr->roamingMngrThresholdsConfig, &pRoamingMngrConfigParams->roamingMngrThresholdsConfig, sizeof(roamingMngrThresholdsConfig_t));
            
            status = apConn_setRoamThresholds(pRoamingMngr->hAPConnection, &pRoamingMngrConfigParams->roamingMngrThresholdsConfig);

            if (pRoamingMngr->roamingMngrConfig.enableDisable && 
                !pRoamingMngrConfigParams->roamingMngrConfig.enableDisable)
            {   /* disable Roaming Manager */
                apConn_unregisterRoamMngrCallb(pRoamingMngr->hAPConnection);
                pRoamingMngr->roamingMngrConfig.enableDisable = ROAMING_DISABLED;
                return (roamingMngr_smEvent((TI_UINT8*)&pRoamingMngr->currentState, ROAMING_EVENT_STOP, pRoamingMngr));
            }
            else if (!pRoamingMngr->roamingMngrConfig.enableDisable && 
                pRoamingMngrConfigParams->roamingMngrConfig.enableDisable)
            {   /* enable Roaming Manager */
                /* Save the Roaming Configuration parameters */
                pRoamingMngr->roamingMngrConfig.enableDisable = pRoamingMngrConfigParams->roamingMngrConfig.enableDisable;
                /* register Roaming callback */
                apConn_registerRoamMngrCallb(pRoamingMngr->hAPConnection, 
                                             roamingMngr_triggerRoamingCb,
                                             roamingMngr_connStatusCb,
                                             roamingMngr_updateNeighborApListCb);
            }
        }
        break;

    case ROAMING_MNGR_USER_DEFINED_TRIGGER:
        {
            TUserDefinedQualityTrigger *pUserTrigger = &pParam->content.rssiSnrTrigger;
            RssiSnrTriggerCfg_t         tTriggerCfg;

            /* Translate User Index to FW Index */
            if (pUserTrigger->uIndex == 0)
            {
                pUserTrigger->uIndex = TRIGGER_EVENT_USER_0;
            }
            else
            {
                pUserTrigger->uIndex = TRIGGER_EVENT_USER_1;
            }

            TRACE8(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION , "roamingMngr_setParam - USER_DEFINED_TRIGGER: \n 									  index = %d, \n									  threshold = %d, \n pacing = %d, \n metric = %d, \n type = %d, \n direction = %d, \n hystersis = %d, \n enable = %d \n",pUserTrigger->uIndex,pUserTrigger->iThreshold,pUserTrigger->uPacing,pUserTrigger->uMetric,pUserTrigger->uType,pUserTrigger->uDirection,pUserTrigger->uHystersis,pUserTrigger->uEnable);
            /* Copy from user structure to driver structure */
            tTriggerCfg.index     = pUserTrigger->uIndex;
            tTriggerCfg.threshold = pUserTrigger->iThreshold;
            tTriggerCfg.pacing    = pUserTrigger->uPacing;
            tTriggerCfg.metric    = pUserTrigger->uMetric;
            tTriggerCfg.type      = pUserTrigger->uType;
            tTriggerCfg.direction = pUserTrigger->uDirection;
            tTriggerCfg.hystersis = pUserTrigger->uHystersis;
            tTriggerCfg.enable    = pUserTrigger->uEnable;

            /* Send used defined trigger to FW (the related FW events are handled by the currBSS). */
            TWD_CfgRssiSnrTrigger (pRoamingMngr->hTWD, &tTriggerCfg);
        }
        break;


    /*********** For Debug Purposes ***********/

    case ROAMING_MNGR_TRIGGER_EVENT:
        /* Enable/disable Internal Roaming */
        TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_setParam TRIGGER_EVENT=  %d \n", pParam->content.roamingTriggerType);
        apConn_reportRoamingEvent(pRoamingMngr->hAPConnection, (apConn_roamingTrigger_e)pParam->content.roamingTriggerType, NULL);
        break;
    
    case ROAMING_MNGR_CONN_STATUS:
        /* External request to connect to BBSID */
        TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_setParam CONN_STATUS=  %d \n", pParam->content.roamingConnStatus);
        roamingMngr_connStatusCb(pRoamingMngr, &pParam->content.roamingConnStatus);
        break;

    default:
        TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, "roamingMngr_setParam bad param=  %X\n", pParam->paramType);

        break;
    }


    return status;
}

TI_STATUS roamingMngr_getParam(TI_HANDLE hRoamingMngr, paramInfo_t *pParam)
{
    roamingMngr_t *pRoamingMngr = (roamingMngr_t*)hRoamingMngr;

    if (pParam == NULL)
    {
        TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR , "roamingMngr_getParam(): pParam is NULL!\n");
        return TI_NOK;
    }

    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_getParam   %X \n", pParam->paramType);

    switch (pParam->paramType)
    {
    case ROAMING_MNGR_APPLICATION_CONFIGURATION:
        {
            roamingMngrConfigParams_t   *pRoamingMngrConfigParams;
    
            pRoamingMngrConfigParams = &pParam->content.roamingConfigBuffer;
    
            if (pRoamingMngr->roamingMngrConfig.enableDisable == ROAMING_DISABLED) 
            {   
                pRoamingMngrConfigParams->roamingMngrConfig.enableDisable = TI_FALSE;
            }
            else 
            {
                pRoamingMngrConfigParams->roamingMngrConfig.enableDisable = TI_TRUE;
            }
            pRoamingMngrConfigParams->roamingMngrConfig.apQualityThreshold = pRoamingMngr->roamingMngrConfig.apQualityThreshold;
            pRoamingMngrConfigParams->roamingMngrConfig.lowPassFilterRoamingAttempt = pRoamingMngr->roamingMngrConfig.lowPassFilterRoamingAttempt;

            apConn_getRoamThresholds(pRoamingMngr->hAPConnection, &pRoamingMngr->roamingMngrThresholdsConfig);
            os_memoryCopy(pRoamingMngr->hOs, &pRoamingMngrConfigParams->roamingMngrThresholdsConfig, &pRoamingMngr->roamingMngrThresholdsConfig, sizeof(roamingMngrThresholdsConfig_t));
            pParam->paramLength = sizeof(roamingMngrConfigParams_t);
        }
        break;

#ifdef TI_DBG
    case ROAMING_MNGR_PRINT_STATISTICS:
        roamingMngr_printStatistics(pRoamingMngr);
        break;

    case ROAMING_MNGR_RESET_STATISTICS:
        roamingMngr_resetStatistics(pRoamingMngr);
        break;

    case ROAMING_MNGR_PRINT_CURRENT_STATUS:
        WLAN_OS_REPORT(("Roaming Current State = %d, enableDisable=%d\n, maskRoamingEvents = %d, roamingTrigger=%d \n scanType=%d, handoverWasPerformed=%d \n, candidateApIndex=%d, lowQualityTriggerTimestamp=%d \n",
                        pRoamingMngr->currentState,
                        pRoamingMngr->roamingMngrConfig.enableDisable,
                        pRoamingMngr->maskRoamingEvents,
                        pRoamingMngr->roamingTrigger,
                        pRoamingMngr->scanType,
                        pRoamingMngr->handoverWasPerformed,
                        pRoamingMngr->candidateApIndex,
                        pRoamingMngr->lowQualityTriggerTimestamp));
        break;
    case ROAMING_MNGR_PRINT_CANDIDATE_TABLE:
        {
            TI_UINT32      index;

            if (pRoamingMngr->pListOfAPs==NULL)
            {
                TRACE0( pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "Roaming Mngr the candidate AP list is invalid \n");
                break;
            }
            TRACE1( pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "The number of candidates is %d\n", pRoamingMngr->pListOfAPs->numOfEntries);

            TRACE1( pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "Roaming Mngr Neighbor AP list, num of candidates = %d\n", pRoamingMngr->listOfCandidateAps.numOfNeighborBSS);

            for (index=0; index<pRoamingMngr->listOfCandidateAps.numOfNeighborBSS; index++)
            {
                TI_UINT32  candidateIndex;
                bssEntry_t  *pBssEntry;

                candidateIndex = pRoamingMngr->listOfCandidateAps.neighborBSSList[index];
                pBssEntry = &pRoamingMngr->pListOfAPs->BSSList[candidateIndex];
                TRACE8( pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "candiate %d, BSSID=%x-%x-%x-%x-%x-%x, RSSI =%d \n", candidateIndex, pBssEntry->BSSID[0], pBssEntry->BSSID[1], pBssEntry->BSSID[2], pBssEntry->BSSID[3], pBssEntry->BSSID[4], pBssEntry->BSSID[5], pBssEntry->RSSI);
            }
            TRACE1( pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "Roaming Mngr Pre-Auth AP list, num of candidates = %d\n", pRoamingMngr->listOfCandidateAps.numOfPreAuthBSS);

            for (index=0; index<pRoamingMngr->listOfCandidateAps.numOfPreAuthBSS; index++)
            {
                TI_UINT32  candidateIndex;
                bssEntry_t  *pBssEntry;

                candidateIndex = pRoamingMngr->listOfCandidateAps.preAuthBSSList[index];
                pBssEntry = &pRoamingMngr->pListOfAPs->BSSList[candidateIndex];
                TRACE8( pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "candiate %d, BSSID=%x-%x-%x-%x-%x-%x, RSSI =%d \n", candidateIndex, pBssEntry->BSSID[0], pBssEntry->BSSID[1], pBssEntry->BSSID[2], pBssEntry->BSSID[3], pBssEntry->BSSID[4], pBssEntry->BSSID[5], pBssEntry->RSSI);
            }
            TRACE1( pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "Roaming Mngr Regular AP list, num of candidates = %d\n", pRoamingMngr->listOfCandidateAps.numOfRegularBSS);

            for (index=0; index<pRoamingMngr->listOfCandidateAps.numOfRegularBSS; index++)
            {
                TI_UINT32  candidateIndex;
                bssEntry_t  *pBssEntry;

                candidateIndex = pRoamingMngr->listOfCandidateAps.regularBSSList[index];
                pBssEntry = &pRoamingMngr->pListOfAPs->BSSList[candidateIndex];
                TRACE8( pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "candiate %d, BSSID=%x-%x-%x-%x-%x-%x, RSSI =%d \n", candidateIndex, pBssEntry->BSSID[0], pBssEntry->BSSID[1], pBssEntry->BSSID[2], pBssEntry->BSSID[3], pBssEntry->BSSID[4], pBssEntry->BSSID[5], pBssEntry->RSSI);
            }
        }
        break;

#endif /*TI_DBG*/

    default:
        TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, "roamingMngr_getParam  bad paramType= %X \n", pParam->paramType);
        return TI_NOK;
    }

    return TI_OK;
}

TI_STATUS roamingMngr_immediateScanComplete(TI_HANDLE hRoamingMngr, scan_mngrResultStatus_e scanCmpltStatus)
{
    roamingMngr_t           *pRoamingMngr;
    roamingMngr_smEvents    roamingEvent;


    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if (pRoamingMngr == NULL)
    {
        return TI_NOK;
    }

    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_immediateScanComplete, scanCmpltStatus = %d\n", 							 scanCmpltStatus);

    if (scanCmpltStatus == SCAN_MRS_SCAN_COMPLETE_OK)
    {   
		/* The scan completed TI_OK, get the updated list of APs */
        pRoamingMngr->pListOfAPs = scanMngr_getBSSList(pRoamingMngr->hScanMngr);
        if ((pRoamingMngr->pListOfAPs != NULL) && (pRoamingMngr->pListOfAPs->numOfEntries > 0))
        {   
			/* APs were found, start selection */
            pRoamingMngr->scanType = ROAMING_NO_SCAN;
            roamingEvent = ROAMING_EVENT_SELECT;
        }
        else
        {   /* There were no APs, if the scan was partial, retry full scan */
            if ((pRoamingMngr->scanType == ROAMING_PARTIAL_SCAN) ||
                (pRoamingMngr->scanType == ROAMING_PARTIAL_SCAN_RETRY))
            {
                pRoamingMngr->scanType = ROAMING_FULL_SCAN;
                roamingEvent = ROAMING_EVENT_SCAN;
            }
            else
            {   
				/* No APs were found in FULL SCAN, report failure */
                roamingEvent = ROAMING_EVENT_SELECT;
            }
        }
    }
	/* scanCmpltStatus != SCAN_MRS_SCAN_COMPLETE_OK */
    else
    {   
		/* The scan failed, retry scanning according to the current scan type */
        pRoamingMngr->pListOfAPs = scanMngr_getBSSList(pRoamingMngr->hScanMngr);
        if ((pRoamingMngr->pListOfAPs != NULL) && (pRoamingMngr->pListOfAPs->numOfEntries > 0))
        {   
			/* APs were found, start selection */
            pRoamingMngr->scanType = ROAMING_NO_SCAN;
            roamingEvent = ROAMING_EVENT_SELECT;
        }
        else
        {   
			/* The scan failed, and there were no APs found. 
			Retry scanning according to the current scan type */
			switch (pRoamingMngr->scanType)
			{
			case ROAMING_PARTIAL_SCAN:
				roamingEvent = ROAMING_EVENT_SCAN;
				pRoamingMngr->scanType = ROAMING_PARTIAL_SCAN_RETRY;
				break;
			case ROAMING_PARTIAL_SCAN_RETRY:
				roamingEvent = ROAMING_EVENT_SELECT;
				pRoamingMngr->scanType = ROAMING_NO_SCAN;
				break;
			case ROAMING_FULL_SCAN:
				roamingEvent = ROAMING_EVENT_SCAN;
				pRoamingMngr->scanType = ROAMING_FULL_SCAN_RETRY;
				break;
			case ROAMING_FULL_SCAN_RETRY:
					roamingEvent = ROAMING_EVENT_SELECT;
				pRoamingMngr->scanType = ROAMING_NO_SCAN;
				break;
			default:
				roamingEvent = ROAMING_EVENT_SELECT;
TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_ERROR, "roamingMngr_immediateScanComplete, pRoamingMngr->scanType = %d\n", 								   pRoamingMngr->scanType);
				pRoamingMngr->scanType = ROAMING_NO_SCAN;       
				break;
			} /* switch (pRoamingMngr->scanType) */
        }
    }

    TRACE2(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_immediateScanComplete, roamingEvent = %d, scanType=%d\n", 							roamingEvent, 							 pRoamingMngr->scanType);

    return (roamingMngr_smEvent((TI_UINT8*)&pRoamingMngr->currentState, roamingEvent, pRoamingMngr));
    
}

TI_STATUS roamingMngr_updateNewBssList(TI_HANDLE hRoamingMngr, bssList_t *bssList)
{

    roamingMngr_t       *pRoamingMngr;

    pRoamingMngr = (roamingMngr_t*)hRoamingMngr;
    if ((pRoamingMngr == NULL) || (bssList == NULL))
    {
        return TI_NOK;
    }

    TRACE1(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_updateNewBssList, number of APs = %d\n", bssList->numOfEntries);

    if (pRoamingMngr->currentState != ROAMING_STATE_WAIT_4_TRIGGER)
    {
        TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_WARNING, "roamingMngr_updateNewBssList, ignore APs when not in WAIT_4_TRIGGER state \n");
        return TI_NOK;
    }


    if (pRoamingMngr->staCapabilities.authMode!=os802_11AuthModeWPA2)
    {   /* No Pre-Auth is required */
        TRACE0(pRoamingMngr->hReport, REPORT_SEVERITY_INFORMATION, "roamingMngr_updateNewBssList, No Pre-Auth is required\n");
        return TI_OK;
    }
    apConn_preAuthenticate(pRoamingMngr->hAPConnection, bssList);

    return TI_OK;

}


