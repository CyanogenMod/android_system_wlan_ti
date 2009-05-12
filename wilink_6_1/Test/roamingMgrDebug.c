/*
 * roamingMgrDebug.c
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

/** \file reportReplvl.c
 *  \brief Report level implementation
 * 
 *  \see reportReplvl.h
 */

/***************************************************************************/
/*																									*/
/*		MODULE:	reportReplvl.c						 										*/
/*    PURPOSE:	Report level implementation	 										*/
/*																									*/
/***************************************************************************/
#include "tidef.h"
#include "report.h"
#include "paramOut.h"
#include "roamingMgrDebug.h"
#include "roamingMngrApi.h"
#include "apConnApi.h"


void printRoamingMgrHelpMenu(void);

/*	Function implementation */
void roamingMgrDebugFunction(TI_HANDLE hRoamingMngr, 
					   TI_UINT32	funcType, 
					   void		*pParam)
{
	paramInfo_t			param;

	
	switch (funcType)
	{
	case ROAMING_MGR_DEBUG_HELP_MENU:
		printRoamingMgrHelpMenu();
		break;

	case PRINT_ROAMING_STATISTICS:
		param.paramType = ROAMING_MNGR_PRINT_STATISTICS;
		roamingMngr_getParam(hRoamingMngr, &param);
		break;

	case RESET_ROAMING_STATISTICS:
		param.paramType = ROAMING_MNGR_RESET_STATISTICS;
		roamingMngr_getParam(hRoamingMngr, &param);
		break;

	case PRINT_ROAMING_CURRENT_STATUS:
		param.paramType = ROAMING_MNGR_PRINT_CURRENT_STATUS;
		roamingMngr_getParam(hRoamingMngr, &param);
		break;

	case PRINT_ROAMING_CANDIDATE_TABLE:
		param.paramType = ROAMING_MNGR_PRINT_CANDIDATE_TABLE;
		roamingMngr_getParam(hRoamingMngr, &param);
		break;

	case TRIGGER_ROAMING_LOW_QUALITY_EVENT:
		param.paramType = ROAMING_MNGR_TRIGGER_EVENT;
		param.content.roamingTriggerType = ROAMING_TRIGGER_LOW_QUALITY;
		roamingMngr_setParam(hRoamingMngr, &param);
		break;

	case TRIGGER_ROAMING_BSS_LOSS_EVENT:
		param.paramType = ROAMING_MNGR_TRIGGER_EVENT;
		param.content.roamingTriggerType = ROAMING_TRIGGER_BSS_LOSS;
		roamingMngr_setParam(hRoamingMngr, &param);
		break;

	case TRIGGER_ROAMING_SWITCH_CHANNEL_EVENT:
		param.paramType = ROAMING_MNGR_TRIGGER_EVENT;
		param.content.roamingTriggerType = ROAMING_TRIGGER_SWITCH_CHANNEL;
		roamingMngr_setParam(hRoamingMngr, &param);
		break;

	case TRIGGER_ROAMING_AP_DISCONNECT_EVENT:
		param.paramType = ROAMING_MNGR_TRIGGER_EVENT;
		param.content.roamingTriggerType = ROAMING_TRIGGER_AP_DISCONNECT;
		roamingMngr_setParam(hRoamingMngr, &param);
		break;

	case TRIGGER_ROAMING_CONNECT_EVENT:
		param.paramType = ROAMING_MNGR_CONN_STATUS;
		param.content.roamingConnStatus = CONN_STATUS_CONNECTED;
		roamingMngr_setParam(hRoamingMngr, &param);
		break;

	case TRIGGER_ROAMING_NOT_CONNECTED_EVENT:
		param.paramType = ROAMING_MNGR_CONN_STATUS;
		param.content.roamingConnStatus = CONN_STATUS_NOT_CONNECTED;
		roamingMngr_setParam(hRoamingMngr, &param);
		break;

	case TRIGGER_ROAMING_HANDOVER_SUCCESS_EVENT:
		param.paramType = ROAMING_MNGR_CONN_STATUS;
		param.content.roamingConnStatus = CONN_STATUS_HANDOVER_SUCCESS;
		roamingMngr_setParam(hRoamingMngr, &param);
		break;

	case TRIGGER_ROAMING_HANDOVER_FAILURE_EVENT:
		param.paramType = ROAMING_MNGR_CONN_STATUS;
		param.content.roamingConnStatus = CONN_STATUS_HANDOVER_FAILURE;
		roamingMngr_setParam(hRoamingMngr, &param);
		break;

	default:
		WLAN_OS_REPORT(("Invalid function type in Debug  Function Command, funcType= %d\n\n", funcType));
		break;
	}
} 


void printRoamingMgrHelpMenu(void)
{
	WLAN_OS_REPORT(("\n\n   Roaming Manager Debug Menu   \n"));
	WLAN_OS_REPORT(("------------------------\n"));


	WLAN_OS_REPORT(("        %02d - ROAMING_MGR_DEBUG_HELP_MENU \n", ROAMING_MGR_DEBUG_HELP_MENU));

	WLAN_OS_REPORT(("        %02d - PRINT_ROAMING_STATISTICS \n", PRINT_ROAMING_STATISTICS));
	WLAN_OS_REPORT(("        %02d - RESET_ROAMING_STATISTICS \n", RESET_ROAMING_STATISTICS));

	WLAN_OS_REPORT(("        %02d - PRINT_ROAMING_CURRENT_STATUS \n", PRINT_ROAMING_CURRENT_STATUS));
	WLAN_OS_REPORT(("        %02d - PRINT_ROAMING_CANDIDATE_TABLE \n", PRINT_ROAMING_CANDIDATE_TABLE));

	WLAN_OS_REPORT(("        %02d - TRIGGER_ROAMING_LOW_QUALITY_EVENT \n", TRIGGER_ROAMING_LOW_QUALITY_EVENT));
	WLAN_OS_REPORT(("        %02d - TRIGGER_ROAMING_BSS_LOSS_EVENT \n", TRIGGER_ROAMING_BSS_LOSS_EVENT));
	WLAN_OS_REPORT(("        %02d - TRIGGER_ROAMING_SWITCH_CHANNEL_EVENT \n", TRIGGER_ROAMING_SWITCH_CHANNEL_EVENT));
	WLAN_OS_REPORT(("        %02d - TRIGGER_ROAMING_AP_DISCONNECT_EVENT \n", TRIGGER_ROAMING_AP_DISCONNECT_EVENT));

	WLAN_OS_REPORT(("        %02d - TRIGGER_ROAMING_CONNECT_EVENT \n", TRIGGER_ROAMING_CONNECT_EVENT));
	WLAN_OS_REPORT(("        %02d - TRIGGER_ROAMING_NOT_CONNECTED_EVENT \n", TRIGGER_ROAMING_NOT_CONNECTED_EVENT));

	WLAN_OS_REPORT(("        %02d - TRIGGER_ROAMING_HANDOVER_SUCCESS_EVENT \n", TRIGGER_ROAMING_HANDOVER_SUCCESS_EVENT));
	WLAN_OS_REPORT(("        %02d - TRIGGER_ROAMING_HANDOVER_FAILURE_EVENT \n", TRIGGER_ROAMING_HANDOVER_FAILURE_EVENT));
	

	WLAN_OS_REPORT(("\n------------------------\n"));
}




