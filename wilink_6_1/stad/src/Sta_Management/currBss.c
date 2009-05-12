/*
 * currBss.c
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

/** \file currBss.c
 *  \brief Current BSS info
 *
 *  \see currBss.h
 */

/****************************************************************************
 *                                                                          *
 *   MODULE:  Current BSS                                                   *
 *   PURPOSE:                                                               *
 *   Roaming ability of eSTA is implemented by Roaming Manager Component and 
 *   described in "Roaming Manager module LLD" document, and by 
 *   AP Connection module. AP Connection module implemented as two sub-modules.
 *   The major one is AP Connection, that is responsible for: 
 *   - providing Roaming Manager with access to other parts of WLAN Driver, 
 *   - implementing low levels of roaming mechanism.
 *   Current BSS sub-module takes care of:
 *   - maintaining database of current AP info,
 *   - providing access to database of current AP info.
 *   The Current BSS represents the BSS we are currently connected to. 
 *   Among other parameters, it holds the capabilities of the current AP, 
 *   its ID and its quality.
 *   When FW indicates 'Out of Sync' event, Current BSS module is responsible
 *   for awaking the device, sending unicast Probe request, waiting for
 *   response and - in case FW comes to the conclusion that there was 
 *   no response - for triggering "Beacon missed" to AP Connection module. 
 *   In eSTA5.0 FW updates and checks the quality of the connection with
 *   current AP. Current BSS module is responsible to handle event of type
 *   'Low RSSI' from FW. Third type of roaming event reported by FW is
 *   'Consecutive no ack on TX", and it is handled as well by Current
 *   BSS module.Upon reception of any roaming event from FW, Current BSS
 *   module is transferring this event to the AP Connection module in case
 *   of BSS connection, or to SME module in case of IBSS connection.
 *   When WLAN driver is working in IBSS mode, Current BSS module is holding
 *   the parameters of BSS (channel, band, SSID etc.).
 *                                                                          *
 ****************************************************************************/

#define __FILE_ID__  FILE_ID_65
#include "currBss.h"
#include "osApi.h"
#include "report.h"
#include "802_11Defs.h"
#include "DataCtrl_Api.h"
#include "qosMngr_API.h"
#include "regulatoryDomainApi.h"
#include "apConn.h"
#include "scanMngrApi.h" 
#include "MacServices_api.h"
#include "smeApi.h"
#include "sme.h"
#include "TWDriver.h"
#include "EvHandler.h"
#include "DrvMainModules.h"
#include "siteMgrApi.h"


/* Constants */
#define TRIGGER_LOW_RSSI_PACING 1000
#define TRIGGER_LOW_SNR_PACING 1000
#define TRIGGER_BG_SCAN_PACING 10
#define TRIGGER_BG_SCAN_HYSTERESIS 3
static const TI_UINT32 KEEP_ALIVE_NULL_DATA_INDEX = 3;

/* Enumerations */


/* Typedefs */

typedef TI_UINT8 (*currBSS_beaconRxCallb_t) (TI_HANDLE hModule, TI_UINT64 staTSF, TI_UINT8 dtimCount);


/* Structures */

/**
* Current BSS control block 
* Following structure defines parameters that can be configured externally,
* internal variables, and handlers of other modules used by Current BSS module
*/
typedef struct _currBSS_t
{
    /* Internal variables and configurable parameters */
    ScanBssType_e type;                   /**< Set by SME module; EBSS, IBSS or none */
    ERadioBand  band;                   /**< Set by SME module */
    TI_UINT8    channel;                /**< Set by AP Connection, SME and Switch Channel modules */
    TI_BOOL     isConnected;            /**< Default: not connected */
    bssEntry_t  currAPInfo;             /**< Set by SME upon request from AP Connection */

    TI_INT8     lowRssiThreshold;       /**< Last configured threshold for Low-RSSI */
    TI_INT8     lowSnrThreshold;        /**< Last configured threshold Low-SNR */
    TI_INT8     lowQualityForBkgrdScan; /**< Indicator used to increase the background scan period when quality is low */
    TI_INT8     highQualityForBkgrdScan;/**< Indicator used to reduce the background scan period when quality is normal */
    TI_UINT8    numExpectedTbttForBSSLoss;/**< last configured value without Soft Gemini compensation */
    TI_UINT8    maxTxRetryThreshold;    /**< last configured threshold for max Tx retry */

    TI_BOOL     bUseSGParams;           /**< Whether to use the Soft Gemini compensation on the roaming triggers (currently: BSS Loss) */
                                        /**< This compensation is needed since BT Activity might over-run beacons                       */
    TI_UINT32   SGcompensationPercent;  /**< the percentage of increasing the TbttForBSSLoss value when SG is enabled */
    TI_UINT8    uDefaultKeepAlivePeriod;/**< The default keep-alive period in seconds */
    TI_UINT8    keepAliveBuffer[ WLAN_WITH_SNAP_QOS_HEADER_MAX_SIZE ];
                                        /**< Buffer to store null-data keep-alive template */

    /* Handlers of other modules used by AP Connection */
    TI_HANDLE   hOs;
    TI_HANDLE   hPowerMngr;
    TI_HANDLE   hAPConn;
    TI_HANDLE   hSme;
    TI_HANDLE   hTWD;
    TI_HANDLE   hMlme;
    TI_HANDLE   hReport;
    TI_HANDLE   hRegulatoryDomain;
    TI_HANDLE   hSiteMgr;
    TI_HANDLE   hScanMngr;
    TI_HANDLE   hEvHandler;
    TI_HANDLE   hTxCtrl;
} currBSS_t;

 
/* Internal functions prototypes */

static void currBSS_lowRssiThrCrossed(currBSS_t *hCurrBSS, TI_UINT8 *data, TI_UINT8 dataLength);
static void currBSS_lowSnrThrCrossed(currBSS_t *hCurrBSS, TI_UINT8 *data, TI_UINT8 dataLength);
static void currBSS_consecTxErrors(currBSS_t *hCurrBSS, TI_UINT8 *data, TI_UINT8 dataLength);
static void currBSS_BackgroundScanQuality(TI_HANDLE hCurrBSS, TI_UINT8 *data, TI_UINT8 dataLength);
static void currBSS_RssiSnrUserTrigger0 (TI_HANDLE hCurrBSS, TI_UINT8 *data, TI_UINT8 dataLength);
static void currBSS_RssiSnrUserTrigger1 (TI_HANDLE hCurrBSS, TI_UINT8 *data, TI_UINT8 dataLength);
static void currBSS_BssLost (currBSS_t *hCurrBSS, TI_UINT8 *data, TI_UINT8 dataLength);
static void currBSS_reportRoamingEvent(currBSS_t *hCurrBSS, apConn_roamingTrigger_e roamingEventType, roamingEventData_u *pRoamingEventData);
static void currBSS_updateBSSLoss(currBSS_t *pCurrBSS);

/* Public functions implementation */


/**
*
* currBSS_create
*
* \b Description: 
*
* Create the Current BSS context: allocate memory for internal variables
*
* \b ARGS:
*
*  I   - hOS - the handle to the OS object
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
TI_HANDLE currBSS_create(TI_HANDLE hOs)
{
    currBSS_t   *pCurrBss;

    if ((pCurrBss = os_memoryAlloc(hOs, sizeof(currBSS_t))) != NULL)
    {
        pCurrBss->hOs = hOs;
    
        return pCurrBss;
    }
    else /* Failed to allocate control block */
    {
        WLAN_OS_REPORT(("FATAL ERROR: currBSS_create(): Error allocating cb - aborting\n"));
        return NULL;
    }
}


/**
*
* currBSS_unload
*
* \b Description: 
*
* Finish Current BSS module work.
*
* \b ARGS:
*
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_unload(TI_HANDLE hCurrBSS)
{
    currBSS_t   *pCurrBSS;
    
    if (hCurrBSS != NULL)
    {
        pCurrBSS = (currBSS_t *)hCurrBSS;

        /* Free pre-allocated control block */
        os_memoryFree(pCurrBSS->hOs, pCurrBSS, sizeof(currBSS_t));
    }
    return TI_OK;
}


/**
*
* currBSS_init
*
* \b Description: 
*
* Get other modules handles.
*
* \b ARGS:
*
*  I   pStadHandles - The driver modules handles
*  
* \b RETURNS:
*
*  void
*
* \sa 
*/
void currBSS_init (TStadHandlesList *pStadHandles)
{
    currBSS_t *pCurrBSS = (currBSS_t *)(pStadHandles->hCurrBss);
        
    pCurrBSS->hAPConn       = pStadHandles->hAPConnection;
    pCurrBSS->hTWD          = pStadHandles->hTWD;
    pCurrBSS->hMlme         = pStadHandles->hMlmeSm;
    pCurrBSS->hPowerMngr    = pStadHandles->hPowerMgr;
    pCurrBSS->hSme          = pStadHandles->hSme;
    pCurrBSS->hSiteMgr      = pStadHandles->hSiteMgr;
    pCurrBSS->hReport       = pStadHandles->hReport;
    pCurrBSS->hScanMngr     = pStadHandles->hScanMngr;
    pCurrBSS->hEvHandler    = pStadHandles->hEvHandler;
    pCurrBSS->hTxCtrl       = pStadHandles->hTxCtrl;
}


/**
*
* currBSS_SetDefaults
*
* \b Description: 
*
* Prepare Current BSS module to work
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_SetDefaults (TI_HANDLE hCurrBSS, TCurrBssInitParams *pInitParams)
{
    currBSS_t *pCurrBSS = (currBSS_t *)hCurrBSS;
    TRroamingTriggerParams params;
    RssiSnrTriggerCfg_t tTriggerCfg;
    
    /* Registration succeeded, continue with init procedure */
    pCurrBSS->band = RADIO_BAND_2_4_GHZ;
    pCurrBSS->channel = 0;
    pCurrBSS->isConnected = TI_FALSE;
    pCurrBSS->type = BSS_ANY;
    pCurrBSS->currAPInfo.RSSI = 0;
    pCurrBSS->bUseSGParams = TI_FALSE;
    pCurrBSS->uDefaultKeepAlivePeriod = pInitParams->uNullDataKeepAlivePeriod; 
    
    /* Configure and enable the Low RSSI, the Low SNR and the Missed beacon events */
    TWD_RegisterEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_0, (void *)currBSS_lowRssiThrCrossed, pCurrBSS); 
    TWD_EnableEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_0);

    TWD_RegisterEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_1, (void *)currBSS_lowSnrThrCrossed, pCurrBSS); 
    TWD_EnableEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_1);

    TWD_RegisterEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_4, (void *)currBSS_BackgroundScanQuality, pCurrBSS); 
    TWD_EnableEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_4);

    TWD_RegisterEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_5, (void *)currBSS_RssiSnrUserTrigger0, pCurrBSS); 
    TWD_EnableEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_5);

    TWD_RegisterEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_6, (void *)currBSS_RssiSnrUserTrigger1, pCurrBSS); 
    TWD_EnableEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_RSSI_SNR_TRIGGER_6);

    pCurrBSS->lowRssiThreshold = RSSI_DEFAULT_THRESHOLD;
    tTriggerCfg.index     = TRIGGER_EVENT_LOW_RSSI;
    tTriggerCfg.threshold = pCurrBSS->lowRssiThreshold;
    tTriggerCfg.pacing    = TRIGGER_LOW_RSSI_PACING;
    tTriggerCfg.metric    = METRIC_EVENT_RSSI_BEACON;
    tTriggerCfg.type      = RX_QUALITY_EVENT_LEVEL;
    tTriggerCfg.direction = RSSI_EVENT_DIR_LOW;
    tTriggerCfg.hystersis = 0;
    tTriggerCfg.enable    = TI_TRUE;
    TWD_CfgRssiSnrTrigger (pCurrBSS->hTWD, &tTriggerCfg);

    pCurrBSS->lowSnrThreshold = SNR_DEFAULT_THRESHOLD;
    tTriggerCfg.index     = TRIGGER_EVENT_LOW_SNR;
    tTriggerCfg.threshold = pCurrBSS->lowSnrThreshold;
    tTriggerCfg.pacing    = TRIGGER_LOW_SNR_PACING;
    tTriggerCfg.metric    = METRIC_EVENT_SNR_BEACON;
    tTriggerCfg.type      = RX_QUALITY_EVENT_LEVEL;
    tTriggerCfg.direction = RSSI_EVENT_DIR_LOW;
    tTriggerCfg.hystersis = 0;
    tTriggerCfg.enable    = TI_TRUE;
    TWD_CfgRssiSnrTrigger (pCurrBSS->hTWD, &tTriggerCfg);

    pCurrBSS->highQualityForBkgrdScan = RSSI_DEFAULT_THRESHOLD;
    pCurrBSS->lowQualityForBkgrdScan = RSSI_DEFAULT_THRESHOLD;
    tTriggerCfg.index     = TRIGGER_EVENT_BG_SCAN;
    tTriggerCfg.threshold = pCurrBSS->lowQualityForBkgrdScan;
    tTriggerCfg.pacing    = TRIGGER_BG_SCAN_PACING;
    tTriggerCfg.metric    = METRIC_EVENT_RSSI_DATA;
    tTriggerCfg.type      = RX_QUALITY_EVENT_EDGE;
    tTriggerCfg.direction = RSSI_EVENT_DIR_BIDIR;
    tTriggerCfg.hystersis = TRIGGER_BG_SCAN_HYSTERESIS;
    tTriggerCfg.enable    = TI_TRUE;
    TWD_CfgRssiSnrTrigger (pCurrBSS->hTWD, &tTriggerCfg);
    
    /* Register for 'BSS-Loss' event */
    TWD_RegisterEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_BSS_LOSE, (void *)currBSS_BssLost, pCurrBSS);
    TWD_EnableEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_BSS_LOSE);

    /* save last configured value for handling Soft Gemini changes */ 
    pCurrBSS->numExpectedTbttForBSSLoss = OUT_OF_SYNC_DEFAULT_THRESHOLD;
    params.TsfMissThreshold = OUT_OF_SYNC_DEFAULT_THRESHOLD;
    params.BssLossTimeout = NO_BEACON_DEFAULT_TIMEOUT;
    TWD_CfgConnMonitParams (pCurrBSS->hTWD, &params);
    
    /* Register for 'Consec. Tx error' */
    TWD_RegisterEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_MAX_TX_RETRY, (void *)currBSS_consecTxErrors, pCurrBSS);
    TWD_EnableEvent (pCurrBSS->hTWD, TWD_OWN_EVENT_MAX_TX_RETRY);

    pCurrBSS->maxTxRetryThreshold = NO_ACK_DEFAULT_THRESHOLD;
    params.maxTxRetry = NO_ACK_DEFAULT_THRESHOLD;
    TWD_CfgMaxTxRetry (pCurrBSS->hTWD, &params);
    
    return TI_OK;
}


/**
*
* currBSS_updateRoamingTriggers
*
* \b Description: 
*
* Configure parameter of Current BSS
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  I   - params - pointer to datablock of roaming threshols \n
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_updateRoamingTriggers (TI_HANDLE hCurrBSS, roamingMngrThresholdsConfig_t *params)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
    TRroamingTriggerParams roamingTriggersParams;
    RssiSnrTriggerCfg_t tTriggerCfg;

    if (pCurrBSS->lowRssiThreshold != params->lowRssiThreshold) 
    {
        pCurrBSS->lowRssiThreshold = params->lowRssiThreshold;

        tTriggerCfg.index     = TRIGGER_EVENT_LOW_RSSI;
        tTriggerCfg.threshold = pCurrBSS->lowRssiThreshold;
        tTriggerCfg.pacing    = TRIGGER_LOW_RSSI_PACING;
        tTriggerCfg.metric    = METRIC_EVENT_RSSI_BEACON;
        tTriggerCfg.type      = RX_QUALITY_EVENT_LEVEL;
        tTriggerCfg.direction = RSSI_EVENT_DIR_LOW;
        tTriggerCfg.hystersis = 0;
        tTriggerCfg.enable    = TI_TRUE;

        TWD_CfgRssiSnrTrigger (pCurrBSS->hTWD, &tTriggerCfg);
    }

    if (pCurrBSS->lowSnrThreshold != params->lowSnrThreshold) 
    {
        pCurrBSS->lowSnrThreshold = params->lowSnrThreshold;

        tTriggerCfg.index     = TRIGGER_EVENT_LOW_SNR;
        tTriggerCfg.threshold = pCurrBSS->lowSnrThreshold;
        tTriggerCfg.pacing    = TRIGGER_LOW_SNR_PACING;
        tTriggerCfg.metric    = METRIC_EVENT_SNR_BEACON;
        tTriggerCfg.type      = RX_QUALITY_EVENT_LEVEL;
        tTriggerCfg.direction = RSSI_EVENT_DIR_LOW;
        tTriggerCfg.hystersis = 0;
        tTriggerCfg.enable    = TI_TRUE;
    
        TWD_CfgRssiSnrTrigger (pCurrBSS->hTWD, &tTriggerCfg);
    }

    if (pCurrBSS->lowQualityForBkgrdScan != params->lowQualityForBackgroungScanCondition) 
    {
        pCurrBSS->lowQualityForBkgrdScan = params->lowQualityForBackgroungScanCondition;
        tTriggerCfg.index     = TRIGGER_EVENT_BG_SCAN;
        tTriggerCfg.threshold = pCurrBSS->lowQualityForBkgrdScan;
        tTriggerCfg.pacing    = TRIGGER_BG_SCAN_PACING;
        tTriggerCfg.metric    = METRIC_EVENT_RSSI_DATA;
        tTriggerCfg.type      = RX_QUALITY_EVENT_EDGE;
        tTriggerCfg.direction = RSSI_EVENT_DIR_BIDIR;
        tTriggerCfg.hystersis = TRIGGER_BG_SCAN_HYSTERESIS;
        tTriggerCfg.enable    = TI_TRUE;

        TWD_CfgRssiSnrTrigger (pCurrBSS->hTWD, &tTriggerCfg);
    }

    if (pCurrBSS->numExpectedTbttForBSSLoss != params->numExpectedTbttForBSSLoss) 
    {
        /* save last configured value for handling Soft Gemini changes */ 
        pCurrBSS->numExpectedTbttForBSSLoss = params->numExpectedTbttForBSSLoss;
        /* Configure TWD with 'No BSS' thresholds (Same as the other parameters but in a special
            function for the Soft Gemini module consideration) */
        currBSS_updateBSSLoss(pCurrBSS);
    }
    
    /* Configure TWD with 'Consecutive NACK' thresholds */
    if (pCurrBSS->maxTxRetryThreshold != params->dataRetryThreshold) 
    {
        pCurrBSS->maxTxRetryThreshold = params->dataRetryThreshold;
        roamingTriggersParams.maxTxRetry = pCurrBSS->maxTxRetryThreshold;
        TWD_CfgMaxTxRetry (pCurrBSS->hTWD, &roamingTriggersParams);
    }

    pCurrBSS->highQualityForBkgrdScan = params->normalQualityForBackgroungScanCondition;
    
    return TI_OK;
}

/**
*
* currBSS_getRoamingParams
*
* \b Description: 
*
* Retrieves the roaming triggers stored in the CurrBSS module.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  O   - aNumExpectedTbttForBSSLoss - Current BSS handle \n
*  O   - aLowQualityForBackgroungScanCondition - Current BSS handle \n
*  O   - aNormalQualityForBackgroungScanCondition - Current BSS handle \n
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_getRoamingParams(TI_HANDLE hCurrBSS,
                                   TI_UINT8 * aNumExpectedTbttForBSSLoss,
                                   TI_INT8 * aLowQualityForBackgroungScanCondition,
                                   TI_INT8 * aNormalQualityForBackgroungScanCondition)
{
    currBSS_t * pCurrBSS = (currBSS_t *) hCurrBSS;

    *aNumExpectedTbttForBSSLoss = pCurrBSS->numExpectedTbttForBSSLoss;
    *aLowQualityForBackgroungScanCondition = pCurrBSS->lowQualityForBkgrdScan;
    *aNormalQualityForBackgroungScanCondition = pCurrBSS->highQualityForBkgrdScan;
    
    return TI_OK;
}

/**
*
* currBSS_SGconfigureBSSLoss
*
* \b Description: 
*
*   This function is called by the Soft Gemini module in order to enable/disable the use of
*   the compensation value for the BSSLoss count , and the percent of increasing that value
*   It also set the new parameter to the FW (with another generic function)
*   The compensation is needed since BT activity might over-run recieved beacons
*    
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*        SGcompensationPercent - percent of increasing the BSSLoss value to the FW \n
*        bUseSGParams - whether to use the SG compensation
*
* \b RETURNS:
*
*  -
*
* \sa 
*/

void currBSS_SGconfigureBSSLoss(TI_HANDLE hCurrBSS,
                                        TI_UINT32 SGcompensationPercent , TI_BOOL bUseSGParams)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
    
    pCurrBSS->bUseSGParams = bUseSGParams;
    pCurrBSS->SGcompensationPercent = SGcompensationPercent;

TRACE1(pCurrBSS->hReport, REPORT_SEVERITY_INFORMATION, "CurrBSS_SGConf: SG =%d\n", pCurrBSS->bUseSGParams);

    /* update the change of BSSLoss in the FW */
    currBSS_updateBSSLoss(pCurrBSS);
}

/**
*
* currBSS_updateBSSLoss
*
* \b Description: 
*
*   This function updates only BSS Loss parameter , we need it to be able to consider the
*   Soft Gemini status , and change the parameter according to it 
*
* \b ARGS:
*
*  I   - pCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  -
*
* \sa 
*/
void currBSS_updateBSSLoss(currBSS_t   *pCurrBSS)
{
    TRroamingTriggerParams roamingTriggersParams;

    /* In Ad-Hoc we use default parameter */
    if (pCurrBSS->type == BSS_INDEPENDENT)
    {
       roamingTriggersParams.TsfMissThreshold = OUT_OF_SYNC_IBSS_THRESHOLD; 
    }
    else /* In Infra we use the saved parameter */
    {
        roamingTriggersParams.TsfMissThreshold = pCurrBSS->numExpectedTbttForBSSLoss;
    }
    
    roamingTriggersParams.BssLossTimeout = NO_BEACON_DEFAULT_TIMEOUT;
    
TRACE2(pCurrBSS->hReport, REPORT_SEVERITY_INFORMATION, ": SG=%d, Band=%d\n", pCurrBSS->bUseSGParams, pCurrBSS->currAPInfo.band);


    /* if Soft Gemini is enabled - increase the BSSLoss value (because BT activity might over-run beacons) */
    if ((pCurrBSS->bUseSGParams) && (pCurrBSS->currAPInfo.band == RADIO_BAND_2_4_GHZ))
    {
        roamingTriggersParams.TsfMissThreshold = (roamingTriggersParams.TsfMissThreshold * 
            (100 + pCurrBSS->SGcompensationPercent)) / 100;

        TRACE2(pCurrBSS->hReport, REPORT_SEVERITY_INFORMATION, ": old value = %d, new value (for SG compensation) = %d\n", pCurrBSS->numExpectedTbttForBSSLoss,roamingTriggersParams.TsfMissThreshold);
    }

    TWD_CfgConnMonitParams (pCurrBSS->hTWD, &roamingTriggersParams);
}

/**
*
* currBSS_swChFinished
*
* \b Description: 
*
* Called when switch channel process is complete in order to reset RSSI calculations
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  -
*
* \sa 
*/
void currBSS_restartRssiCounting(TI_HANDLE hCurrBSS)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;

    pCurrBSS->currAPInfo.RSSI = 0;
}

/**
*
* currBSS_getBssInfo
*
* \b Description: 
*
* Get parameter of Current BSS
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  pointer to current BSS info block.
*
* \sa 
*/
bssEntry_t *currBSS_getBssInfo(TI_HANDLE hCurrBSS)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;

    /* Return pointer to current AP info */
    return &(pCurrBSS->currAPInfo);
}


/**
*
* currBSS_probRespReceivedCallb
*
* \b Description: 
*
* Callback function, provided to MLME module. Called each time Probe response received.
* This function verifies that the Probe response was sent by current AP, and then
* updates current AP database.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_probRespReceivedCallb(TI_HANDLE hCurrBSS,
                                        TRxAttr *pRxAttr,
                                        TMacAddr *bssid,
                                        mlmeFrameInfo_t *pFrameInfo,
										TI_UINT8 *dataBuffer,
                                        TI_UINT16 bufLength)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
    paramInfo_t param;
    
    param.paramType = SITE_MGR_CURRENT_BSSID_PARAM;
    siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    

    if (pCurrBSS->isConnected && MAC_EQUAL (param.content.siteMgrDesiredBSSID, *bssid))
    {
        siteMgr_updateSite(pCurrBSS->hSiteMgr, bssid, pFrameInfo, pRxAttr->channel, (ERadioBand)pRxAttr->band, TI_FALSE);
        /* Save the IE part of the Probe Response buffer in the site table */
        siteMgr_saveProbeRespBuffer(pCurrBSS->hSiteMgr, bssid, (TI_UINT8 *)dataBuffer, bufLength);
    }
    return TI_OK;
}



/**
*
* currBSS_beaconReceivedCallb
*
* \b Description: 
*
* Callback function, provided to MLME module. Called each time Beacon received.
* This function verifies that the Probe response was sent by current AP, and then
* updates current AP database.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_beaconReceivedCallb(TI_HANDLE hCurrBSS,
                                      TRxAttr *pRxAttr,
                                      TMacAddr *bssid,
                                      mlmeFrameInfo_t *pFrameInfo,
                                      TI_UINT8 *dataBuffer,
                                      TI_UINT16 bufLength)
{
    currBSS_t           *pCurrBSS = (currBSS_t *)hCurrBSS;
    paramInfo_t         param;
    ScanBssType_enum    bssType;

    bssType = ((pFrameInfo->content.iePacket.capabilities >> CAP_ESS_SHIFT) & CAP_ESS_MASK) ? BSS_INFRASTRUCTURE : BSS_INDEPENDENT;
    param.paramType = SITE_MGR_CURRENT_BSSID_PARAM;
    siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    

    if (pCurrBSS->isConnected && MAC_EQUAL(param.content.siteMgrDesiredBSSID, *bssid))
    {
        siteMgr_updateSite(pCurrBSS->hSiteMgr, bssid, pFrameInfo, pRxAttr->channel, (ERadioBand)pRxAttr->band, TI_FALSE);
        /* Save the IE part of the beacon buffer in the site table */
        siteMgr_saveBeaconBuffer(pCurrBSS->hSiteMgr, bssid, (TI_UINT8 *)dataBuffer, bufLength);
    }
	else if(pCurrBSS->isConnected && (bssType==BSS_INDEPENDENT))
    {
		siteMgr_IbssMerge(pCurrBSS->hSiteMgr, param.content.siteMgrDesiredBSSID, *bssid,
						  pFrameInfo, pRxAttr->channel, (ERadioBand)pRxAttr->band);
		siteMgr_saveBeaconBuffer(pCurrBSS->hSiteMgr, bssid, (TI_UINT8 *)dataBuffer, bufLength);
	}

    return TI_OK;
}


/**
*
* currBSS_updateConnectedState
*
* \b Description: 
*
* This function is called when FW recovery performed.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  I   - isConnected - TI_TRUE or TI_FALSE \n
*  I   - type - IBSS or EBSS \n
*  
* \b RETURNS:
*
*  -
*
* \sa 
*/
void currBSS_updateConnectedState(TI_HANDLE hCurrBSS, TI_BOOL isConnected, ScanBssType_e type)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
    paramInfo_t  param;

    pCurrBSS->type = type;
    pCurrBSS->isConnected = isConnected;

    if (isConnected) 
    {
        /*** Store the info of current AP ***/

        /* BSSID */
        param.paramType = SITE_MGR_CURRENT_BSSID_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    
        MAC_COPY (pCurrBSS->currAPInfo.BSSID, param.content.siteMgrDesiredBSSID);

        /* Rx rate */
        param.paramType = SITE_MGR_LAST_RX_RATE_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        pCurrBSS->currAPInfo.rxRate = param.content.ctrlDataCurrentBasicRate;

        /* Band */
        param.paramType = SITE_MGR_RADIO_BAND_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        pCurrBSS->currAPInfo.band = param.content.siteMgrRadioBand;

        /* Channel */
        param.paramType = SITE_MGR_CURRENT_CHANNEL_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        pCurrBSS->currAPInfo.channel = param.content.siteMgrCurrentChannel;

        /* Last Rx Tsf */
        param.paramType = SITE_MGR_CURRENT_TSF_TIME_STAMP;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        os_memoryCopy(pCurrBSS->hOs, &pCurrBSS->currAPInfo.lastRxTSF, 
                      param.content.siteMgrCurrentTsfTimeStamp, sizeof(pCurrBSS->currAPInfo.lastRxTSF));

        /* Beacon interval */
        param.paramType = SITE_MGR_BEACON_INTERVAL_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        pCurrBSS->currAPInfo.beaconInterval = param.content.beaconInterval;

        /* Capability */
        param.paramType = SITE_MGR_SITE_CAPABILITY_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr,&param);
        pCurrBSS->currAPInfo.capabilities = param.content.siteMgrSiteCapability;
        param.paramType = SITE_MGR_CURRENT_TSF_TIME_STAMP;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    

        /* pCurrBSS->currAPInfo.lastRxHostTimestamp = *((TI_UINT64 *)(pIEs->TimeStamp));*/ /* TBD*/
        os_memoryCopy(pCurrBSS->hOs, &pCurrBSS->currAPInfo.lastRxHostTimestamp, param.content.siteMgrCurrentTsfTimeStamp, sizeof(TI_UINT32));

        param.paramType = SITE_MGR_LAST_BEACON_BUF_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        pCurrBSS->currAPInfo.pBuffer = param.content.siteMgrLastBeacon.buffer;
        pCurrBSS->currAPInfo.bufferLength = param.content.siteMgrLastBeacon.bufLength;
        pCurrBSS->currAPInfo.resultType = (param.content.siteMgrLastBeacon.isBeacon) ? SCAN_RFT_BEACON : SCAN_RFT_PROBE_RESPONSE;

        /* Set BSS Loss to Fw - note that it depends on the Connection type - (Infa/IBSS) */
        currBSS_updateBSSLoss(pCurrBSS);

        if(type == BSS_INFRASTRUCTURE)
        {
            TI_UINT32           uKeepAlivePreiod = pCurrBSS->uDefaultKeepAlivePeriod * 1000; /* convert to ms */
            TSetTemplate        tKeepAliveTemplate;
            TKeepAliveParams    tKeepAliveParams;

            /*
             * only configure the null-data keepa-live message if the interval is valid
             * (either the default interval or the one from teh XCC IE)
             */
            if (0 != uKeepAlivePreiod)
            {
                TRACE0(pCurrBSS->hReport, REPORT_SEVERITY_INFORMATION , "currBSS_updateConnectedState: Configuring null-data keep-alive");
    
                /* build null-data template */
                tKeepAliveTemplate.ptr = &(pCurrBSS->keepAliveBuffer[ 0 ]);
                if ( TI_OK != txCtrlServ_buildNullFrame (pCurrBSS->hTxCtrl, 
                                                         tKeepAliveTemplate.ptr, &(tKeepAliveTemplate.len)))
                {
                    TRACE0(pCurrBSS->hReport, REPORT_SEVERITY_ERROR , "currBSS_updateConnectedState: error building null data frame\n");
    
                }
    
                /* configure null-data template */
                tKeepAliveTemplate.type = KEEP_ALIVE_TEMPLATE;
                tKeepAliveTemplate.index = KEEP_ALIVE_NULL_DATA_INDEX;
                tKeepAliveTemplate.uRateMask = RATE_MASK_UNSPECIFIED;
                TWD_CmdTemplate (pCurrBSS->hTWD, &tKeepAliveTemplate, NULL, NULL);
    
                /* configure paramters */
                tKeepAliveParams.index = KEEP_ALIVE_NULL_DATA_INDEX;
                tKeepAliveParams.enaDisFlag = TI_TRUE; /* enabled */
                tKeepAliveParams.trigType = KEEP_ALIVE_TRIG_TYPE_NO_TX;
                tKeepAliveParams.interval = uKeepAlivePreiod;
                TWD_CfgKeepAlive (pCurrBSS->hTWD, &tKeepAliveParams);
            }
        }        
    }
    else
    {
        if(type == BSS_INFRASTRUCTURE)
        {
            TKeepAliveParams    tKeepAliveParams;

            /* disable NULL-data keep-palive template */
            tKeepAliveParams.index = KEEP_ALIVE_NULL_DATA_INDEX;
            tKeepAliveParams.enaDisFlag = TI_FALSE; /* disabled */
            tKeepAliveParams.interval = 1000; /* minimum accepted by the FW */
            tKeepAliveParams.trigType = KEEP_ALIVE_TRIG_TYPE_NO_TX;
            TWD_CfgKeepAlive (pCurrBSS->hTWD, &tKeepAliveParams);

        }        
    }
}


/**
*
* currBSS_BssLost
*
* \b Description: 
*
* Callback function, called upon BSS-Loss event from FW.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  void
*
* \sa 
*/
static void currBSS_BssLost (currBSS_t *hCurrBSS,
                             TI_UINT8  *data,
                             TI_UINT8   dataLength)
{
    currBSS_reportRoamingEvent(hCurrBSS, ROAMING_TRIGGER_BSS_LOSS, NULL);
}


/**
*
* currBSS_consecTxErrors
*
* \b Description: 
*
* Callback function, provided to HAL module.
*
* \b ARGS:
*
*  I   - pCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
static void currBSS_consecTxErrors(currBSS_t *hCurrBSS,
                                   TI_UINT8     *data,
                                   TI_UINT8     dataLength)
{
    currBSS_reportRoamingEvent(hCurrBSS, ROAMING_TRIGGER_MAX_TX_RETRIES, NULL);
}


/**
*
* currBSS_lowRssiThrCrossed
*
* \b Description: 
*
* Callback function, provided to HAL module.
*
* \b ARGS:
*
*  I   - pCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
static void currBSS_lowRssiThrCrossed(currBSS_t *hCurrBSS,
                                      TI_UINT8     *data,
                                      TI_UINT8     dataLength)
{
    currBSS_reportRoamingEvent(hCurrBSS, ROAMING_TRIGGER_LOW_QUALITY, NULL);
}


/**
*
* currBSS_lowSnrThrCrossed
*
* \b Description: 
*
* Callback function, provided to HAL module.
*
* \b ARGS:
*
*  I   - pCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
static void currBSS_lowSnrThrCrossed(currBSS_t *hCurrBSS,
                                      TI_UINT8     *data,
                                      TI_UINT8     dataLength)
{
    currBSS_reportRoamingEvent(hCurrBSS, ROAMING_TRIGGER_LOW_SNR, NULL);
}

/**
*
* currBSS_reportRoamingEvent
*
* \b Description: 
*
* This function checks the mode of Current BSS module. 
* If connected to EBSS, it reports roaming event to AP Connection.
*
* \b ARGS:
*
*  I   - pCurrBSS - Current BSS handle \n
*  I   - roamingEventType - Roaming trigger to report \n
*  
* \b RETURNS:
*
*  TI_OK on success, TI_NOK on failure.
*
* \sa 
*/
static void currBSS_reportRoamingEvent(currBSS_t *pCurrBSS, 
                                       apConn_roamingTrigger_e roamingEventType,
                                       roamingEventData_u *pRoamingEventData)
{
    TRACE1(pCurrBSS->hReport, REPORT_SEVERITY_INFORMATION, "currBSS_reportRoamingEvent: trigger %d\n", roamingEventType);

    if (pCurrBSS->isConnected)
    {
        if (pCurrBSS->type == BSS_INFRASTRUCTURE) 
        {
            apConn_reportRoamingEvent(pCurrBSS->hAPConn, roamingEventType, pRoamingEventData);
        }
        else /* IBSS */
        { 
            if( roamingEventType == ROAMING_TRIGGER_BSS_LOSS )
            {
                /* If in IBSS call the SME restart function, this logic issues a DISCONNECT 
                 * event and tries to connect to other STA or establish self connection.
                 */
                sme_Restart (pCurrBSS->hSme);
            }
        }
    }
}


/**
*
* currBSS_GetDefaultKeepAlivePeriod
*
* \b Description: 
*
* Get DefaultKeepAlivePeriod parameter value.
*
* \b ARGS:
*
*  I   - hCurrBSS           - Current BSS handle \n
*  I   - uDefaultKeepAlivePeriod - The  value \n
*  
* \b RETURNS:
*
*  None.
*
* \sa 
*/
void currBSS_GetDefaultKeepAlivePeriod (TI_HANDLE hCurrBSS, TI_UINT8* uKeepAlivePeriod)
{ 
    currBSS_t *pCurrBSS = (currBSS_t *)hCurrBSS;

    *uKeepAlivePeriod = pCurrBSS->uDefaultKeepAlivePeriod;
}


/**
*
* currBSS_BackgroundScanQuality
*
* \b Description: 
*
* Called be EventMBox upon Background Scan Quality Trigger.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  None.
*
* \sa 
*/
static void currBSS_BackgroundScanQuality(TI_HANDLE hCurrBSS,
                                      TI_UINT8     *data,
                                      TI_UINT8     dataLength)
{
    currBSS_t *pCurrBSS = (currBSS_t *)hCurrBSS;
    TI_UINT8 averageRssi = *data;
    paramInfo_t param;

    TRACE1(pCurrBSS->hReport, REPORT_SEVERITY_INFORMATION, "BackgroundScanQuality Event: RSSI = %d\n", averageRssi );

    /* Report to AP Connection about reaching RSSI low or normal (high) threshold */
    if (averageRssi < pCurrBSS->lowQualityForBkgrdScan)
    {
        apConn_reportRoamingEvent(pCurrBSS->hAPConn, ROAMING_TRIGGER_LOW_QUALITY_FOR_BG_SCAN, NULL);
    }
    else
    {
        apConn_reportRoamingEvent(pCurrBSS->hAPConn, ROAMING_TRIGGER_NORMAL_QUALITY_FOR_BG_SCAN, NULL); 
    }

    /* Update RSSI: */
    pCurrBSS->currAPInfo.RSSI = averageRssi;

    /* Update Site Table in order to represent the RSSI of current AP correctly in the utility */
    param.paramType = SITE_MGR_CURRENT_SIGNAL_PARAM;
    param.content.siteMgrCurrentSignal.rssi = averageRssi;
    siteMgr_setParam(pCurrBSS->hSiteMgr, &param);

}


/** 
 * \fn     currBSS_RssiSnrUserTrigger0 & 1 
 * \brief  User Defined Trigger 0 & 1 callbacks
 * 
 * Called by EventMBox upon User Defined Trigger 0 or 1.
 * 
 * \note    
 * \param  hCurrBSS   - Current BSS handle
 * \param  data       - The event data
 * \param  dataLength - The event data length
 * \return void  
* \sa 
*/
static void currBSS_RssiSnrUserTrigger0 (TI_HANDLE hCurrBSS, TI_UINT8 *data, TI_UINT8 dataLength)
{
    currBSS_t *pCurrBSS = (currBSS_t *)hCurrBSS;
    EvHandlerSendEvent(pCurrBSS->hEvHandler, IPC_EVENT_RSSI_SNR_TRIGGER_0, data, dataLength);
}

static void currBSS_RssiSnrUserTrigger1 (TI_HANDLE hCurrBSS, TI_UINT8 *data, TI_UINT8 dataLength)
{
    currBSS_t *pCurrBSS = (currBSS_t *)hCurrBSS;
    EvHandlerSendEvent(pCurrBSS->hEvHandler, IPC_EVENT_RSSI_SNR_TRIGGER_1, data, dataLength);
}


