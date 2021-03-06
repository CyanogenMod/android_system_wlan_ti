/** \file mainSecSm.c
 *  \brief 802.1X finite state machine header file
 *
 *  \see mainSecSm.h
 */

/****************************************************************************
**+-----------------------------------------------------------------------+**
**|                                                                       |**
**| Copyright(c) 1998 - 2008 Texas Instruments. All rights reserved.      |**
**| All rights reserved.                                                  |**
**|                                                                       |**
**| Redistribution and use in source and binary forms, with or without    |**
**| modification, are permitted provided that the following conditions    |**
**| are met:                                                              |**
**|                                                                       |**
**|  * Redistributions of source code must retain the above copyright     |**
**|    notice, this list of conditions and the following disclaimer.      |**
**|  * Redistributions in binary form must reproduce the above copyright  |**
**|    notice, this list of conditions and the following disclaimer in    |**
**|    the documentation and/or other materials provided with the         |**
**|    distribution.                                                      |**
**|  * Neither the name Texas Instruments nor the names of its            |**
**|    contributors may be used to endorse or promote products derived    |**
**|    from this software without specific prior written permission.      |**
**|                                                                       |**
**| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   |**
**| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     |**
**| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR |**
**| A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  |**
**| OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |**
**| SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      |**
**| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, |**
**| DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY |**
**| THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   |**
**| (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE |**
**| OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  |**
**|                                                                       |**
**+-----------------------------------------------------------------------+**
****************************************************************************/

/***************************************************************************/
/*                                                                         */
/*      MODULE: mainSecSm.c                                                */
/*    PURPOSE:  Main Security State Machine API                            */
/*                                                                         */
/***************************************************************************/

#include "osApi.h"

#include "paramOut.h"
/*#include "paramIn.h"*/

#include "utils.h"
#include "report.h"

#include "DataCtrl_Api.h"
#include "smeApi.h"

#include "rsn.h"
#include "rsnApi.h"

#include "mainSecSm.h"
#include "mainSecNull.h"


#include "mainSecKeysOnly.h"
#include "mainKeysSm.h"

/* Constants */

/** number of events in the state machine */
#define MAIN_SEC_MAX_NUM_EVENTS     7

/** number of states in the state machine */
#define MAIN_SEC_MAX_NUM_STATES     6

/* Enumerations */

/* Typedefs */

/* Structures */

/* External data definitions */

/* External functions definitions */

/* Global variables */

/* Local function prototypes */

TI_STATUS mainSec_setKey(struct _mainSec_t *pMainSec, securityKeys_t *pKey);
TI_STATUS mainSec_removeKey(struct _mainSec_t *pMainSec, securityKeys_t *pKey);
TI_STATUS mainSec_setDefaultKeyId(struct _mainSec_t *pMainSec, UINT8 keyId);

/* functions */

/**
*
* mainSec_create
*
* \b Description: 
*
* Allocate memory for the main security context, and create all the rest of the needed contexts.
*
* \b ARGS:
*
*  I - hOs - OS handle for OS operations.
*
* \b RETURNS:
*
*  pointer to main security context. If failed, returns NULL.
*
* \sa 
*/
mainSec_t* mainSec_create(TI_HANDLE hOs)
{
    mainSec_t   *pHandle;
    TI_STATUS       status;

    /* allocate association context memory */
    pHandle = (mainSec_t*)os_memoryAlloc(hOs, sizeof(mainSec_t));
    if (pHandle == NULL)
    {
        return NULL;
    }

    os_memoryZero(hOs, pHandle, sizeof(mainSec_t));

    /* allocate memory for association state machine */
    status = fsm_Create(hOs, &pHandle->pMainSecSm, MAIN_SEC_MAX_NUM_STATES, MAIN_SEC_MAX_NUM_EVENTS);
    if (status != OK)
    {
        os_memoryFree(hOs, pHandle, sizeof(mainSec_t));
        return NULL;
    }

    pHandle->pMainKeys = mainKeys_create(hOs);
    if (pHandle->pMainKeys == NULL)
    {
        fsm_Unload(hOs, pHandle->pMainSecSm);
        os_memoryFree(hOs, pHandle, sizeof(mainSec_t));
        return NULL;
    }

    pHandle->pKeyParser = pHandle->pMainKeys->pKeyParser;

    
    pHandle->hOs = hOs;
    
    return pHandle;
}

/**
*
* mainSec_config
*
* \b Description: 
*
* Init main security state machine state machine
*
* \b ARGS:
*
*  none
*
* \b RETURNS:
*
*  OK on success, NOK otherwise.
*
* \sa 
*/
TI_STATUS mainSec_config(mainSec_t *pMainSec, 
                      mainSecInitData_t *pInitData, 
                      void *pParent,
                      TI_HANDLE hReport,
                      TI_HANDLE hOs,
                      TI_HANDLE hCtrlData,
                      TI_HANDLE hEvHandler,
                      TI_HANDLE hConn,
                      TI_HANDLE hHalCtrl   )
{
    TI_STATUS               status;

    pMainSec->setKey = mainSec_setKey;
    pMainSec->removeKey = mainSec_removeKey;
    pMainSec->setDefaultKeyId = mainSec_setDefaultKeyId;

    pMainSec->pParent = pParent;
    pMainSec->hReport = hReport;
    pMainSec->hOs = hOs;

    WLAN_REPORT_SM(pMainSec->hReport,RSN_MODULE_LOG, 
				   ("MainSec SM: config, authProtocol = %d, keyExchangeProtocol=%d, unicastSuite=%d, broadcastSuite=%d\n", 
                    pInitData->pPaeConfig->authProtocol, pInitData->pPaeConfig->keyExchangeProtocol, 
                    pInitData->pPaeConfig->unicastSuite, pInitData->pPaeConfig->broadcastSuite));

    switch (pInitData->pPaeConfig->keyExchangeProtocol)
    {
    case RSN_KEY_MNG_NONE:
        status = mainSecSmNull_config(pMainSec, pInitData->pPaeConfig);
        break;
    case RSN_KEY_MNG_802_1X:
        status = mainSecKeysOnly_config(pMainSec, pInitData->pPaeConfig);
        break;
    default:
        status = mainSecSmNull_config(pMainSec, pInitData->pPaeConfig);
        break;
    }

    status  = mainKeys_config(pMainSec->pMainKeys, pInitData->pPaeConfig, pMainSec, pMainSec->hReport, pMainSec->hOs, 
                           hCtrlData, hEvHandler, hConn, pMainSec->pParent);
    if (status != OK)
    {
        WLAN_REPORT_ERROR(pMainSec->hReport, RSN_MODULE_LOG,
                          ("MAIN_SEC_SM: error in configuring mainKeys SM\n"));
        return status;
    }

    WLAN_REPORT_SM(pMainSec->hReport, RSN_MODULE_LOG,
                          ("MAIN_SEC_SM: successful configuration SM\n"));

    return status;
}

/**
*
* mainSec_config
*
* \b Description: 
*
* Init main security state machine state machine
*
* \b ARGS:
*
*  none
*
* \b RETURNS:
*
*  OK on success, NOK otherwise.
*
* \sa 
*/
TI_STATUS mainSec_unload(mainSec_t *pMainSec)
{
    TI_STATUS   status;

    if (pMainSec == NULL)
    {
        return NOK;
    }

    status = mainKeys_unload(pMainSec->pMainKeys);
    if (status != OK)
    {
        /* report failure but don't stop... */
        WLAN_REPORT_ERROR(pMainSec->hReport, RSN_MODULE_LOG,
                          ("MAIN_SEC_SM: Error releasing Main Keys SM memory \n"));
    }

    status = fsm_Unload(pMainSec->hOs, pMainSec->pMainSecSm);
    if (status != OK)
    {
        /* report failure but don't stop... */
        WLAN_REPORT_ERROR(pMainSec->hReport, RSN_MODULE_LOG,
                          ("MAIN_SEC_SM: Error releasing FSM memory \n"));
    }

    os_memoryFree(pMainSec->hOs, pMainSec, sizeof(mainSec_t));

    return OK;
}

/**
*
* mainSec_setKey
*
* \b Description: 
*
* Start the NULL main security SM. Reports success to the rsn module immediately.
*
* \b ARGS:
*
*  none
*
* \b RETURNS:
*
*  OK on success, NOK otherwise.
*
* \sa 
*/
TI_STATUS mainSec_setKey(struct _mainSec_t *pMainSec, securityKeys_t *pKey)
{
    TI_STATUS               status = OK;
    
    if ((pMainSec == NULL) || (pKey == NULL))
    {
        return NOK;
    }

    if (pKey->keyType != NULL_KEY)
    {
        WLAN_REPORT_INFORMATION(pMainSec->hReport, RSN_MODULE_LOG,
                                ("MAIN_SEC_SM: setting key #%d, value = 0x%X 0x%X 0x%X 0x%X 0x%X\n",
                                 pKey->keyIndex, (UINT8)pKey->encKey[0],
                                 (UINT8)pKey->encKey[1],
                                 (UINT8)pKey->encKey[2],
                                 (UINT8)pKey->encKey[3],
                                 (UINT8)pKey->encKey[4]));

        status = pMainSec->pParent->setKey(pMainSec->pParent, pKey);
    }
    
    return status;
}

/**
*
* mainSec_removeKey
*
* \b Description: 
*
* Start the NULL main security SM. Reports success to the rsn module immediately.
*
* \b ARGS:
*
*  none
*
* \b RETURNS:
*
*  OK on success, NOK otherwise.
*
* \sa 
*/
TI_STATUS mainSec_removeKey(struct _mainSec_t *pMainSec, securityKeys_t *pKey)
{
    TI_STATUS               status = OK;
    
    if ((pMainSec == NULL) || (pKey == NULL))
    {
        return NOK;
    }

    if (pKey->keyType != NULL_KEY)
    {
        WLAN_REPORT_INFORMATION(pMainSec->hReport, RSN_MODULE_LOG,
                                ("MAIN_SEC_SM: removing key #%d, \n",
                                 pKey->keyIndex));

        status = pMainSec->pParent->removeKey(pMainSec->pParent, pKey);
    }
    
    return status;
}

/**
*
* mainSec_setDefaultKeyId
*
* \b Description: 
*
* Start the NULL main security SM. Reports success to the rsn module immediately.
*
* \b ARGS:
*
*  none
*
* \b RETURNS:
*
*  OK on success, NOK otherwise.
*
* \sa 
*/
TI_STATUS mainSec_setDefaultKeyId(struct _mainSec_t *pMainSec, UINT8 keyId)
{
    TI_STATUS               status = OK;

    if (pMainSec == NULL)
    {
        return NOK;
    }

    status = pMainSec->pParent->setDefaultKeyId(pMainSec->pParent, keyId);
    
    return status;
}


