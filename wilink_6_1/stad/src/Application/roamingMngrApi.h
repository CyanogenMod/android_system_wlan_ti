/*
 * roamingMngrApi.h
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

/** \file roamingMngrApi.h
 *  \brief Internal Roaming Manager API
 *
 *  \see roamingMngr.c
 */

/****************************************************************************
 *                                                                          *
 *   MODULE:  Roaming Manager	    		                                *
 *   PURPOSE: Roaming Manager Module API                              		*
 *                                                                          *
 ****************************************************************************/

#ifndef _ROAMING_MNGR_API_H_
#define _ROAMING_MNGR_API_H_

/*#include "802_11Defs.h"*/
#include "osApi.h"
#include "paramOut.h"
#include "scanMngrApi.h"
#include "bssTypes.h"
#include "DrvMainModules.h"

/*-----------*/
/* Constants */
/*-----------*/

/*--------------*/
/* Enumerations */
/*--------------*/

/*----------*/
/* Typedefs */
/*----------*/

/*------------*/
/* Structures */
/*------------*/

/*---------------------------*/
/* External data definitions */
/*---------------------------*/

/*--------------------------------*/
/* External functions definitions */
/*--------------------------------*/

/*----------------------------*/
/* Global Function prototypes */
/*----------------------------*/

/**
 * \brief Create the Roaming Manager context
 * 
 * \param  hOs	  	- OS handler
 * \return	A pointer to the roaming manager handler on success, 
 * 			NULL on failure (unable to allocate memory or other system resource) 
 * 
 * \par Description
 * Creates the Roaming Manager context: \n
 * Allocate control block for preconfigured parameters and internal variables, create state-machine
 * 
 * \sa	roamingMngr_unload
 */ 
TI_HANDLE roamingMngr_create(TI_HANDLE hOs);
/**
 * \brief Configure the Roaming Manager module
 * 
 * \param  pStadHandles	  	- The driver modules handles
 * \return void 
 * 
 * \par Description
 * Configure the Roaming Manager module to do the following:
 * - Initialize the Station broadcast key State Machine matrix
 * - Store handlers of other modules (report module, Scan Manager, AP connection, TWD)
 * - Initialize the roaming manager internal variables
 * - Configure the roaming manager state-machine
 * 
 * \sa	
 */ 
void roamingMngr_init (TStadHandlesList *pStadHandles);
/**
 * \brief Unloads the Roaming Manager Module
 * 
 * \param  hRoamingMngr - Roaming Manager handler
 * \return TI_OK on success or TI_NOK on failure 
 * 
 * \par Description
 * Unloads the components of the Roaming Manager module from memory:	\n 
 * releases the allocation for the state-machine and internal variables
 * 
 * \sa	roamingMngr_create
 */ 
TI_STATUS roamingMngr_unload(TI_HANDLE hRoamingMngr);
/**
 * \brief Get Roaming Manager parameters from the roamingMngr SM
 * 
 * \param  hRoamingMngr - Roaming Manager handler
 * \param  pParam 		- Output Parameters
 * \return TI_OK on success or TI_NOK on failure 
 * 
 * \par Description
 * 
 * \sa
 */ 
TI_STATUS roamingMngr_getParam(TI_HANDLE hRoamingMngr, paramInfo_t *pParam);
/**
 * \brief Set Roaming Manager Module parameters to the roamingMngr SM
 * 
 * \param  hRoamingMngr - Roaming Manager handler
 * \param  pParam 		- Input Parameters
 * \return TI_OK on success or TI_NOK on failure 
 * 
 * \par Description
 * 
 * \sa
 */ 
TI_STATUS roamingMngr_setParam(TI_HANDLE hRoamingMngr, paramInfo_t *pParam);
/**
 * \brief  Indicates Roaming Manager that an Immediate Scan was completed 
 *			and provides it with the Scan result
 * 
 * \param  hRoamingMngr  	- Handle to the roaming manager
 * \param  scanCmpltStatus	- Scan complete reason
 * \return TI_OK on success or TI_NOK on failure 
 * 
 * \par Description
 * This procedure is called when the Scan Manager completed Immediate Scan for Roaming.
 * It performs the following:
 * - Partial or Full scan
 * - Re-try Partial or Full scan if the previous scan failed
 * - Full scan if the previous partial scan didn't get any APs
 * - Fail event if all the Scans failed
 * 
 * Algorithm description:
 * - If Scan Complete is OK: 
 * -------------------------
 *		- If APs found:
 * 			- starts Selection
 *		- If NO APs found:
 * 			- If Previous Scan was Partial:
 *				- Perform Full Scan
 *			- If Previous Scan was Full:
 *				- Report Error
 *
 * - If Scan Complete is NOT OK: 
 * -----------------------------
 * - Re-Try Scan
 *		- If APs found:
 * 			- starts Selection
 * 		- If NO APs found:
 *			- Re-Try Scan with current Scan Type (Partial/Full Scan Retry or No Scan)
 * 
 * \sa
 */ 
TI_STATUS roamingMngr_immediateScanComplete(TI_HANDLE hRoamingMngr, scan_mngrResultStatus_e scanCmpltStatus);
/**
 * \brief  Indicates that a new BSSID is added to the BSS table
 * 
 * \param  hRoamingMngr  	- Handle to the roaming manager
 * \param  newBss_entry	  	- List of BSSIDs that have been found
 * \return TI_OK on success or TI_NOK on failure 
 * 
 * \par Description
 * Indicates that a new BSSID is added to the BSS table (Called by the Scan Manager when new BSSID was found). 
 * This function triggers preauthentication to the new BSS.
 * 
 * \sa
 */ 
TI_STATUS roamingMngr_updateNewBssList(TI_HANDLE hRoamingMngr, bssList_t *newBss_entry);


#endif /*  _ROAMING_MNGR_API_H_*/

