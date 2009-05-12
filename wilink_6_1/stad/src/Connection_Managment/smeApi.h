/*
 * smeApi.h
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

/** \file smeMgr.h
 *  \brief SME interface
 *
 *  
 */

/****************************************************************************/
/*																			*/
/*		MODULE:		smeApi.h												*/
/*		PURPOSE:	SME interface to Other core modules						*/
/*																			*/
/****************************************************************************/
#ifndef __SME_API_H__
#define __SME_API_H__

#include "tidef.h"
#include "paramOut.h"
#include "mlmeApi.h"

/* Typedefs */
typedef enum
{
	NO_MATCH = 0,
	MATCH		= 1
} match_e;


/* Prototypes */

TI_STATUS conn_reportMlmeStatus(TI_HANDLE			hConn, 
							mgmtStatus_e		status,
							TI_UINT16				uStatusCode);

TI_STATUS conn_reportRsnStatus(TI_HANDLE			hConn, 
							mgmtStatus_e		status);

TI_STATUS siteMgr_updateSite(TI_HANDLE			hSiteMgr, 
						  TMacAddr		*bssid, 
						  mlmeFrameInfo_t	*pFrameInfo,
						  TI_UINT8				rxChannel,
                          ERadioBand       band,
						  TI_BOOL				measuring);

TI_STATUS siteMgr_IbssMerge(TI_HANDLE       hSiteMgr,
                          TMacAddr      	our_bssid,
						  TMacAddr      	new_bssid,
                          mlmeFrameInfo_t   *pFrameInfo,
                          TI_UINT8          rxChannel,
                          ERadioBand        band);

TI_STATUS siteMgr_saveProbeRespBuffer(TI_HANDLE hSiteMgr, TMacAddr	*bssid, TI_UINT8 *pProbeRespBuffer, TI_UINT32 length);

TI_STATUS siteMgr_saveBeaconBuffer(TI_HANDLE hSiteMgr, TMacAddr *bssid, TI_UINT8 *pBeaconBuffer, TI_UINT32 length);


#endif /* __SME_API_H__ */
