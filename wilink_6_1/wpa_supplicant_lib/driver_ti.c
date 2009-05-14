/* 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include "includes.h"
#include <sys/ioctl.h>
#include <net/if_arp.h>
#include <net/if.h>

#include "wireless_copy.h"
#include "common.h"
#include "driver.h"
#include "l2_packet.h"
#include "eloop.h"
#include "wpa_supplicant.h"
#include "priv_netlink.h"
#include "driver_wext.h"
#include "wpa.h"
#include "wpa_ctrl.h"

#define TI_WLAN_WILINK_6_SUPPORT

#ifdef TI_WLAN_WILINK_6_SUPPORT
#include "cu_ostypes.h"	    
#include "STADExternalIf.h"
#include "convert.h"
#endif

#ifdef CONFIG_CLIENT_MLME
#include <netpacket/packet.h>
#include <hostapd_ioctl.h>
#include <ieee80211_common.h>
/* from net/mac80211.h */
enum {
	MODE_IEEE80211A = 0 /* IEEE 802.11a */,
	MODE_IEEE80211B = 1 /* IEEE 802.11b only */,
	MODE_ATHEROS_TURBO = 2 /* Atheros Turbo mode (2x.11a at 5 GHz) */,
	MODE_IEEE80211G = 3 /* IEEE 802.11g (and 802.11b compatibility) */,
	MODE_ATHEROS_TURBOG = 4 /* Atheros Turbo mode (2x.11g at 2.4 GHz) */,
	NUM_IEEE80211_MODES = 5
};

#include "mlme.h"

#ifndef ETH_P_ALL
#define ETH_P_ALL 0x0003
#endif
#endif /* CONFIG_CLIENT_MLME */

#define TIWLAN_DRV_NAME         "tiwlan0"

typedef enum {
    BLUETOOTH_COEXISTENCE_MODE_ENABLED = 0,
    BLUETOOTH_COEXISTENCE_MODE_DISABLED,
    BLUETOOTH_COEXISTENCE_MODE_SENSE
} EUIBTCoexMode;


struct wpa_driver_tista_data {
	void *ctx;
	int event_sock;
	int ioctl_sock;
	int mlme_sock;
	char ifname[IFNAMSIZ + 1];
	int ifindex;
	int ifindex2;
	u8 *assoc_req_ies;
	size_t assoc_req_ies_len;
	u8 *assoc_resp_ies;
	size_t assoc_resp_ies_len;
	struct wpa_driver_capa capa;
	int has_capability;
	int we_version_compiled;

	/* for set_auth_alg fallback */
	int use_crypt;
	int auth_alg_fallback;

	int operstate;

	char mlmedev[IFNAMSIZ + 1];

	int scan_complete_events;
#ifdef TI_WLAN_WILINK_6_SUPPORT
	u8 own_addr[ETH_ALEN];          /* MAC address of WLAN interface */
	int driver_is_loaded;           /* TRUE/FALSE flag if driver is already loaded and can be accessed */
	int scan_type;                  /* SCAN_TYPE_NORMAL_ACTIVE or  SCAN_TYPE_NORMAL_PASSIVE */
	int scan_channels;              /* Number of allowed scan channels */
	unsigned int link_speed;        /* Link Speed */
	u32 btcoex_mode;                /* BtCoex Mode */
#endif
};


static int wpa_driver_tista_flush_pmkid(void *priv);
static int wpa_driver_tista_get_range(void *priv);
void wpa_driver_tista_scan_timeout(void *eloop_ctx, void *timeout_ctx);
int wpa_driver_tista_alternative_ifindex(struct wpa_driver_tista_data *drv,
					const char *ifname);
int wpa_driver_tista_get_ssid(void *priv, u8 *ssid);
int wpa_driver_tista_set_mode(void *priv, int mode);

#ifdef TI_WLAN_WILINK_6_SUPPORT
static int wpa_driver_tista_private_send( void *priv, u32 ioctl_cmd, void *bufIn, u32 sizeIn, void *bufOut, u32 sizeOut )
{
	struct wpa_driver_tista_data *drv = (struct wpa_driver_tista_data *)priv;
	ti_private_cmd_t private_cmd;
	struct iwreq iwr;
	s32 res;

	private_cmd.cmd = ioctl_cmd;
	if(bufOut == NULL)
	    private_cmd.flags = PRIVATE_CMD_SET_FLAG;
	else
	    private_cmd.flags = PRIVATE_CMD_GET_FLAG;

	private_cmd.in_buffer = bufIn;
	private_cmd.in_buffer_len = sizeIn;
	private_cmd.out_buffer = bufOut;
	private_cmd.out_buffer_len = sizeOut;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);

	iwr.u.data.pointer = &private_cmd;
	iwr.u.data.length = sizeof(ti_private_cmd_t);
	iwr.u.data.flags = 0;

	res = ioctl(drv->ioctl_sock, SIOCIWFIRSTPRIV, &iwr);
	if(res != 0)
	{
	    wpa_printf(MSG_ERROR, "ERROR - wpa_driver_tista_private_send - error sending Wext private IOCTL to STA driver (ioctl_cmd = %x,  res = %d, errno = %d)", 
			         ioctl_cmd, res, errno);
	    return -1;
	}

	wpa_printf(MSG_DEBUG, "wpa_driver_tista_private_send ioctl_cmd = %x  res = %d", ioctl_cmd, res);

	return 0;
}

static int wpa_driver_tista_driver_start( void *priv )
{
	struct wpa_driver_tista_data *drv = (struct wpa_driver_tista_data *)priv;
	u32 uDummyBuf;
	s32 res;

	res = wpa_driver_tista_private_send(priv, DRIVER_START_PARAM, &uDummyBuf, sizeof(uDummyBuf), NULL, 0);

	if(0 != res)
		wpa_printf(MSG_ERROR, "ERROR - Failed to start driver!");
	else
		wpa_printf(MSG_DEBUG, "wpa_driver_tista_driver_start success");

	return res;
}

static int wpa_driver_tista_driver_stop( void *priv )
{
	struct wpa_driver_tista_data *drv = (struct wpa_driver_tista_data *)priv;
	u32 uDummyBuf;
	s32 res;

	res = wpa_driver_tista_private_send(priv, DRIVER_STOP_PARAM, &uDummyBuf, sizeof(uDummyBuf), NULL, 0);

	if(0 != res)
		wpa_printf(MSG_ERROR, "ERROR - Failed to stop driver!");
	else
		wpa_printf(MSG_DEBUG, "wpa_driver_tista_driver_stop success");

	return res;
}

static void ti_init_scan_params( scan_Params_t *pScanParams,
                                 int scanType, int noOfChan )
{
    u8 i,j;
    /* init application scan default params */
    pScanParams->desiredSsid.len = 0;
    pScanParams->scanType = scanType;
    pScanParams->band = RADIO_BAND_2_4_GHZ;
    pScanParams->probeReqNumber = 3;
    pScanParams->probeRequestRate = RATE_MASK_UNSPECIFIED; /* Let the FW select */;
    pScanParams->numOfChannels = noOfChan;
    for ( i = 0; i < 14; i++ )
    {
        for ( j = 0; j < 6; j++ )
        {
            pScanParams->channelEntry[ i ].normalChannelEntry.bssId[ j ] = 0xff;
        }
        pScanParams->channelEntry[ i ].normalChannelEntry.earlyTerminationEvent = SCAN_ET_COND_DISABLE;
        pScanParams->channelEntry[ i ].normalChannelEntry.ETMaxNumOfAPframes = 0;
        pScanParams->channelEntry[ i ].normalChannelEntry.maxChannelDwellTime = 60000;
        pScanParams->channelEntry[ i ].normalChannelEntry.minChannelDwellTime = 30000;
        pScanParams->channelEntry[ i ].normalChannelEntry.txPowerDbm = DEF_TX_POWER;
        pScanParams->channelEntry[ i ].normalChannelEntry.channel = i + 1;
    }
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_scan
Routine Description: request scan from driver
Arguments: 
   priv - pointer to private data structure
   ssid - ssid buffer
   ssid_len - length of ssid
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_scan( void *priv, const u8 *ssid, size_t ssid_len )
{
	struct wpa_driver_tista_data *drv = (struct wpa_driver_tista_data *)priv;
	scan_Params_t scanParams;
	s32 res;

	/* If driver is not initialized yet - we cannot access it so return */
	if(!(drv->driver_is_loaded))
		return -1;

	os_memset(&scanParams, 0, sizeof(scan_Params_t));

	/* Initialize scan parameters */
	ti_init_scan_params(&scanParams, drv->scan_type, drv->scan_channels);

    if (ssid && ssid_len > 0 && ssid_len <= sizeof(scanParams.desiredSsid.str)) {
        os_memcpy(scanParams.desiredSsid.str, ssid, ssid_len);
        scanParams.desiredSsid.len = ssid_len;
    }

	res = wpa_driver_tista_private_send(priv, TIWLN_802_11_START_APP_SCAN_SET, &scanParams, sizeof(scanParams), NULL, 0);

	if(0 != res)
		wpa_printf(MSG_ERROR, "ERROR - Failed to do tista scan!");
	else
		wpa_printf(MSG_DEBUG, "wpa_driver_tista_scan success");

	return res;
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_get_mac_addr
Routine Description: return WLAN MAC address
Arguments: 
   priv - pointer to private data structure
Return Value: pointer to BSSID
-----------------------------------------------------------------------------*/
const u8 *wpa_driver_tista_get_mac_addr( void *priv )
{
	struct wpa_driver_tista_data *drv = (struct wpa_driver_tista_data *)priv;
	u8 mac[ETH_ALEN];

	if(0 != wpa_driver_tista_private_send(priv, CTRL_DATA_MAC_ADDRESS, NULL, 0,
		mac, ETH_ALEN))
	{
		wpa_printf(MSG_ERROR, "ERROR - Failed to get mac address!");
		os_memset(drv->own_addr, 0, ETH_ALEN);
	}
	else
	{
		os_memcpy(drv->own_addr, mac, ETH_ALEN);
		wpa_printf(MSG_DEBUG, "Macaddr = " MACSTR, MAC2STR(drv->own_addr));
	}
	wpa_printf(MSG_DEBUG, "wpa_driver_tista_get_mac_addr success");

	return (const u8 *)&drv->own_addr ;
}

static int wpa_driver_tista_get_rssi(void *priv, int *rssi_data, int *rssi_beacon)
{
	struct wpa_driver_tista_data *drv = (struct wpa_driver_tista_data *)priv;
	TCuCommon_RoamingStatisticsTable buffer;

	if(0 != wpa_driver_tista_private_send(priv, TIWLN_802_11_RSSI, NULL, 0,
		&buffer, sizeof(TCuCommon_RoamingStatisticsTable)))
	{
		wpa_printf(MSG_ERROR, "ERROR - Failed to get rssi level");
		*rssi_data = 0;
		*rssi_beacon = 0;
		return -1;
	}
	*rssi_data = (s8)buffer.rssi;
	*rssi_beacon = (s8)buffer.rssiBeacon;
	wpa_printf(MSG_DEBUG, "wpa_driver_tista_get_rssi data %d beacon %d success", *rssi_data, *rssi_beacon);

	return 0;
}

static int wpa_driver_tista_config_power_management(void *priv, TPowerMgr_PowerMode *mode, u8 is_set)
{
	struct wpa_driver_tista_data *drv = (struct wpa_driver_tista_data *)priv;

	if(is_set) // set power mode
	{
		if((mode->PowerMode) < POWER_MODE_MAX)
		{
			if(0 != wpa_driver_tista_private_send(priv, TIWLN_802_11_POWER_MODE_SET, 
				mode, sizeof(TPowerMgr_PowerMode), NULL, 0))
			{
				wpa_printf(MSG_ERROR, "ERROR - Failed to set power mode");
				return -1;
			}
		}
		else
		{
			wpa_printf(MSG_ERROR, "ERROR - Invalid Power Mode");
			return -1;
		}
	}
	else // get power mode
	{
		if(0 != wpa_driver_tista_private_send(priv, TIWLN_802_11_POWER_MODE_GET, NULL, 0,
			mode, sizeof(TPowerMgr_PowerMode)))
		{
			wpa_printf(MSG_ERROR, "ERROR - Failed to get power mode");
			return -1;
		}
	}
	wpa_printf(MSG_DEBUG, "wpa_driver_tista_config_power_management success");

	return 0;
}

static int wpa_driver_tista_enable_bt_coe(void *priv, u32 mode)
{
	struct wpa_driver_tista_data *drv = (struct wpa_driver_tista_data *)priv;
	u32 mode_set = mode;

	/* Mapping the mode between UI enum and driver enum */
	switch(mode_set)
	{
		case BLUETOOTH_COEXISTENCE_MODE_ENABLED:
		case BLUETOOTH_COEXISTENCE_MODE_SENSE:
			mode_set = SG_OPPORTUNISTIC;
			break;
		case BLUETOOTH_COEXISTENCE_MODE_DISABLED:
			mode_set = SG_DISABLE;
			break;
		default:
			wpa_printf(MSG_DEBUG, "wpa_driver_tista_enable_bt_coe - Unknown Mode");
			return -1;
			break;
	}

	if(0 != wpa_driver_tista_private_send(priv, SOFT_GEMINI_SET_ENABLE, 
		&mode_set, sizeof(u32), NULL, 0))
	{
		wpa_printf(MSG_ERROR, "ERROR - Failed to enable BtCoe");
		return -1;
	}
	wpa_printf(MSG_DEBUG, "wpa_driver_tista_enable_bt_coe success");

	return 0;
}

static int wpa_driver_tista_get_bt_coe_status(void *priv, u32 *mode)
{
	struct wpa_driver_tista_data *drv = (struct wpa_driver_tista_data *)priv;
	u32 mode_get;

	if(0 != wpa_driver_tista_private_send(priv, SOFT_GEMINI_GET_CONFIG, NULL, 0,
		&mode_get, sizeof(u32)))
	{
		wpa_printf(MSG_ERROR, "ERROR - Failed to get bt coe status");
		return -1;
	}
	*mode = mode_get;
	wpa_printf(MSG_DEBUG, "wpa_driver_tista_get_bt_coe_status mode %d success", *mode);

	return 0;
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_driver_cmd
Routine Description: executes driver-specific commands
Arguments: 
   priv - pointer to private data structure
   cmd - command
   buf - return buffer
   buf_len - buffer length
Return Value: actual buffer length - success, -1 - failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_driver_cmd( void *priv, char *cmd, char *buf, size_t buf_len )
{
	struct wpa_driver_tista_data *drv = (struct wpa_driver_tista_data *)priv;
	int ret = -1, prev_events;

	wpa_printf(MSG_DEBUG, "%s %s", __func__, cmd);

	if( os_strcasecmp(cmd, "start") == 0 ) {
		wpa_printf(MSG_DEBUG,"Start command");
		ret = wpa_driver_tista_driver_start(priv);
		if( ret == 0 ) {
			drv->driver_is_loaded = TRUE;
			wpa_msg(drv->ctx, MSG_INFO, WPA_EVENT_DRIVER_STATE "STARTED");
		}
	}
	else if( os_strcasecmp(cmd, "stop") == 0 ) {
		wpa_printf(MSG_DEBUG,"Stop command");
		ret = wpa_driver_tista_driver_stop(priv);
		if( ret == 0 ) {
			drv->driver_is_loaded = FALSE;
			wpa_msg(drv->ctx, MSG_INFO, WPA_EVENT_DRIVER_STATE "STOPPED");
		}
	}
	else if( os_strcasecmp(cmd, "macaddr") == 0 ) {
		wpa_driver_tista_get_mac_addr(priv);
		wpa_printf(MSG_DEBUG, "Macaddr command");
		ret = sprintf(buf, "Macaddr = " MACSTR "\n", MAC2STR(drv->own_addr));
		buf[ret]='\0';
		wpa_printf(MSG_DEBUG, "buf %s", buf);
	}
	else if( os_strcasecmp(cmd, "scan-passive") == 0 ) {
		wpa_printf(MSG_DEBUG,"Scan Passive command");
		drv->scan_type =  SCAN_TYPE_NORMAL_PASSIVE;
		ret = 0;
	}
	else if( os_strcasecmp(cmd, "scan-active") == 0 ) {
		wpa_printf(MSG_DEBUG,"Scan Active command");
		drv->scan_type =  SCAN_TYPE_NORMAL_ACTIVE;
		ret = 0;
	}
	else if( os_strcasecmp(cmd, "linkspeed") == 0 ) {
		wpa_printf(MSG_DEBUG,"Link Speed command");
		ret = sprintf(buf,"LinkSpeed %u\n", drv->link_speed);
		buf[ret]='\0';
		wpa_printf(MSG_DEBUG, "buf %s", buf);
	}
	else if( os_strncasecmp(cmd, "scan-channels", 13) == 0 ) {
		int noOfChan;

		noOfChan = atoi(cmd + 13);
		wpa_printf(MSG_DEBUG,"Scan Channels command = %d", noOfChan);
		if( (noOfChan > 0) && (noOfChan <= MAX_NUMBER_OF_CHANNELS_PER_SCAN) )
			drv->scan_channels = noOfChan;
		ret = sprintf(buf,"Scan-Channels = %d\n", drv->scan_channels);
		buf[ret]='\0';
		wpa_printf(MSG_DEBUG, "buf %s", buf);
	}
	else if( os_strcasecmp(cmd, "rssi") == 0 ) {
		u8 ssid[MAX_SSID_LEN];
		int rssi_data, rssi_beacon, len;

		wpa_printf(MSG_DEBUG,"rssi command");

		ret = wpa_driver_tista_get_rssi(priv, &rssi_data, &rssi_beacon);
		if( ret == 0 ) {
			len = wpa_driver_tista_get_ssid( priv, (u8 *)ssid );
			wpa_printf(MSG_DEBUG,"rssi_data %d rssi_beacon %d", rssi_data, rssi_beacon);
			if( (len > 0) && (len <= MAX_SSID_LEN) ) {
				os_memcpy( (void *)buf, (void *)ssid, len );
				ret = len;
				ret += sprintf(&buf[ret], " rssi %d\n", rssi_beacon);
				buf[ret]='\0';
				wpa_printf(MSG_DEBUG, "buf %s", buf);
			}
			else
			{
				wpa_printf(MSG_DEBUG, "Fail to get ssid when reporting rssi");
				ret = -1;
			}
		}
	}
	else if( os_strncasecmp(cmd, "powermode", 9) == 0 ) {
		u32 mode;
		TPowerMgr_PowerMode tMode;

		mode = (u32)atoi(cmd + 9);
		wpa_printf(MSG_DEBUG,"Power Mode command = %u", mode);
		if( mode < POWER_MODE_MAX )
		{
			tMode.PowerMode = (PowerMgr_PowerMode_e)mode;
			tMode.PowerMngPriority = POWER_MANAGER_USER_PRIORITY;
			ret = wpa_driver_tista_config_power_management( priv, &tMode, 1 );
		}
	}
	else if (os_strncasecmp(cmd, "getpower", 8) == 0 ) {
		u32 mode;
		TPowerMgr_PowerMode tMode;

		ret = wpa_driver_tista_config_power_management( priv, &tMode, 0 );
		if( ret == 0 ) {
			ret = sprintf(buf, "powermode = %u\n", tMode.PowerMode);
			buf[ret]='\0';
			wpa_printf(MSG_DEBUG, "buf %s", buf);
		}
	}
	else if( os_strncasecmp(cmd, "btcoexmode", 10) == 0 ) {
		u32 mode;

		mode = (u32)atoi(cmd + 10);
		wpa_printf(MSG_DEBUG,"BtCoex Mode command = %u", mode);
		ret = wpa_driver_tista_enable_bt_coe( priv, mode );
		if( ret == 0 ) {
			drv->btcoex_mode = mode;
		}
	}
	else if( os_strcasecmp(cmd, "btcoexstat") == 0 ) {
		u32 status = drv->btcoex_mode;

		wpa_printf(MSG_DEBUG,"BtCoex Status");
		ret = wpa_driver_tista_get_bt_coe_status( priv, &status );
		if( ret == 0 ) {
			ret = sprintf(buf, "btcoexstatus = 0x%x\n", status);
			buf[ret]='\0';
			wpa_printf(MSG_DEBUG, "buf %s", buf);
		}
	}
	else {
		wpa_printf(MSG_DEBUG,"Unsupported command");
	}


	if(ret == 0)
	{
		os_memcpy(buf, "OK\n", 3);
		ret = 3;
	}

	return ret;
}
#endif

static int wpa_driver_tista_send_oper_ifla(struct wpa_driver_tista_data *drv,
					  int linkmode, int operstate)
{
	struct {
		struct nlmsghdr hdr;
		struct ifinfomsg ifinfo;
		char opts[16];
	} req;
	struct rtattr *rta;
	static int nl_seq;
	ssize_t ret;

	req.hdr.nlmsg_len = NLMSG_LENGTH(sizeof(struct ifinfomsg));
	req.hdr.nlmsg_type = RTM_SETLINK;
	req.hdr.nlmsg_flags = NLM_F_REQUEST;
	req.hdr.nlmsg_seq = ++nl_seq;
	req.hdr.nlmsg_pid = 0;

	req.ifinfo.ifi_family = AF_UNSPEC;
	req.ifinfo.ifi_type = 0;
	req.ifinfo.ifi_index = drv->ifindex;
	req.ifinfo.ifi_flags = 0;
	req.ifinfo.ifi_change = 0;

	if (linkmode != -1) {
		rta = (struct rtattr *)
			((char *) &req + NLMSG_ALIGN(req.hdr.nlmsg_len));
		rta->rta_type = IFLA_LINKMODE;
		rta->rta_len = RTA_LENGTH(sizeof(char));
		*((char *) RTA_DATA(rta)) = linkmode;
		req.hdr.nlmsg_len = NLMSG_ALIGN(req.hdr.nlmsg_len) +
			RTA_LENGTH(sizeof(char));
	}
	if (operstate != -1) {
		rta = (struct rtattr *)
			((char *) &req + NLMSG_ALIGN(req.hdr.nlmsg_len));
		rta->rta_type = IFLA_OPERSTATE;
		rta->rta_len = RTA_LENGTH(sizeof(char));
		*((char *) RTA_DATA(rta)) = operstate;
		req.hdr.nlmsg_len = NLMSG_ALIGN(req.hdr.nlmsg_len) +
			RTA_LENGTH(sizeof(char));
	}

	wpa_printf(MSG_DEBUG, "WEXT: Operstate: linkmode=%d, operstate=%d",
		   linkmode, operstate);

	ret = send(drv->event_sock, &req, req.hdr.nlmsg_len, 0);
	if (ret < 0) {
		wpa_printf(MSG_DEBUG, "WEXT: Sending operstate IFLA failed: "
			   "%s (assume operstate is not supported)",
			   strerror(errno));
	}

	return ret < 0 ? -1 : 0;
}


static int wpa_driver_tista_set_auth_param(struct wpa_driver_tista_data *drv,
					  int idx, u32 value)
{
	struct iwreq iwr;
	int ret = 0;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	iwr.u.param.flags = idx & IW_AUTH_INDEX;
	iwr.u.param.value = value;

	if (ioctl(drv->ioctl_sock, SIOCSIWAUTH, &iwr) < 0) {
		perror("ioctl[SIOCSIWAUTH]");
		fprintf(stderr, "WEXT auth param %d value 0x%x - ",
			idx, value);
		ret = errno == EOPNOTSUPP ? -2 : -1;
	}

	return ret;
}


/**
 * wpa_driver_tista_get_bssid - Get BSSID, SIOCGIWAP
 * @priv: Pointer to private wext data from wpa_driver_tista_init()
 * @bssid: Buffer for BSSID
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_tista_get_bssid(void *priv, u8 *bssid)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iwreq iwr;
	int ret = 0;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);

	if (ioctl(drv->ioctl_sock, SIOCGIWAP, &iwr) < 0) {
		perror("ioctl[SIOCGIWAP]");
		ret = -1;
	}
	os_memcpy(bssid, iwr.u.ap_addr.sa_data, ETH_ALEN);

	return ret;
}


/**
 * wpa_driver_tista_set_bssid - Set BSSID, SIOCSIWAP
 * @priv: Pointer to private wext data from wpa_driver_tista_init()
 * @bssid: BSSID
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_tista_set_bssid(void *priv, const u8 *bssid)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iwreq iwr;
	int ret = 0;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	iwr.u.ap_addr.sa_family = ARPHRD_ETHER;
	if (bssid)
		os_memcpy(iwr.u.ap_addr.sa_data, bssid, ETH_ALEN);
	else
		os_memset(iwr.u.ap_addr.sa_data, 0, ETH_ALEN);

	if (ioctl(drv->ioctl_sock, SIOCSIWAP, &iwr) < 0) {
		perror("ioctl[SIOCSIWAP]");
		ret = -1;
	}

	return ret;
}


/**
 * wpa_driver_tista_get_ssid - Get SSID, SIOCGIWESSID
 * @priv: Pointer to private wext data from wpa_driver_tista_init()
 * @ssid: Buffer for the SSID; must be at least 32 bytes long
 * Returns: SSID length on success, -1 on failure
 */
int wpa_driver_tista_get_ssid(void *priv, u8 *ssid)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iwreq iwr;
	int ret = 0;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	iwr.u.essid.pointer = (caddr_t) ssid;
	iwr.u.essid.length = 32;

	if (ioctl(drv->ioctl_sock, SIOCGIWESSID, &iwr) < 0) {
		perror("ioctl[SIOCGIWESSID]");
		ret = -1;
	} else {
		ret = iwr.u.essid.length;
		if (ret > 32)
			ret = 32;
		/* Some drivers include nul termination in the SSID, so let's
		 * remove it here before further processing. WE-21 changes this
		 * to explicitly require the length _not_ to include nul
		 * termination. */
		if (ret > 0 && ssid[ret - 1] == '\0' &&
		    drv->we_version_compiled < 21)
			ret--;
	}

	return ret;
}


/**
 * wpa_driver_tista_set_ssid - Set SSID, SIOCSIWESSID
 * @priv: Pointer to private wext data from wpa_driver_tista_init()
 * @ssid: SSID
 * @ssid_len: Length of SSID (0..32)
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_tista_set_ssid(void *priv, const u8 *ssid, size_t ssid_len)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iwreq iwr;
	int ret = 0;
	char buf[33];

	if (ssid_len > 32)
		return -1;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	/* flags: 1 = ESSID is active, 0 = not (promiscuous) */
	iwr.u.essid.flags = (ssid_len != 0);
	os_memset(buf, 0, sizeof(buf));
	os_memcpy(buf, ssid, ssid_len);
	iwr.u.essid.pointer = (caddr_t) buf;
	if (drv->we_version_compiled < 21) {
		/* For historic reasons, set SSID length to include one extra
		 * character, C string nul termination, even though SSID is
		 * really an octet string that should not be presented as a C
		 * string. Some Linux drivers decrement the length by one and
		 * can thus end up missing the last octet of the SSID if the
		 * length is not incremented here. WE-21 changes this to
		 * explicitly require the length _not_ to include nul
		 * termination. */
		if (ssid_len)
			ssid_len++;
	}
	iwr.u.essid.length = ssid_len;

	if (ioctl(drv->ioctl_sock, SIOCSIWESSID, &iwr) < 0) {
		perror("ioctl[SIOCSIWESSID]");
		ret = -1;
	}

	return ret;
}


/**
 * wpa_driver_tista_set_freq - Set frequency/channel, SIOCSIWFREQ
 * @priv: Pointer to private wext data from wpa_driver_tista_init()
 * @freq: Frequency in MHz
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_tista_set_freq(void *priv, int freq)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iwreq iwr;
	int ret = 0;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	iwr.u.freq.m = freq * 100000;
	iwr.u.freq.e = 1;

	if (ioctl(drv->ioctl_sock, SIOCSIWFREQ, &iwr) < 0) {
		perror("ioctl[SIOCSIWFREQ]");
		ret = -1;
	}

	return ret;
}


static void
wpa_driver_tista_event_wireless_custom(struct wpa_driver_tista_data *drv, void *ctx, char *custom)
{
	union wpa_event_data data;
#ifdef TI_WLAN_WILINK_6_SUPPORT
	IPC_EV_DATA * pData = NULL;
#endif

	wpa_printf(MSG_MSGDUMP, "WEXT: Custom wireless event: '%s'",
		   custom);

	os_memset(&data, 0, sizeof(data));
	/* Host AP driver */
	if (os_strncmp(custom, "MLME-MICHAELMICFAILURE.indication", 33) == 0) {
		data.michael_mic_failure.unicast =
			os_strstr(custom, " unicast ") != NULL;
		/* TODO: parse parameters(?) */
		wpa_supplicant_event(ctx, EVENT_MICHAEL_MIC_FAILURE, &data);
	} else if (os_strncmp(custom, "ASSOCINFO(ReqIEs=", 17) == 0) {
		char *spos;
		int bytes;

		spos = custom + 17;

		bytes = strspn(spos, "0123456789abcdefABCDEF");
		if (!bytes || (bytes & 1))
			return;
		bytes /= 2;

		data.assoc_info.req_ies = os_malloc(bytes);
		if (data.assoc_info.req_ies == NULL)
			return;

		data.assoc_info.req_ies_len = bytes;
		hexstr2bin(spos, data.assoc_info.req_ies, bytes);

		spos += bytes * 2;

		data.assoc_info.resp_ies = NULL;
		data.assoc_info.resp_ies_len = 0;

		if (os_strncmp(spos, " RespIEs=", 9) == 0) {
			spos += 9;

			bytes = strspn(spos, "0123456789abcdefABCDEF");
			if (!bytes || (bytes & 1))
				goto done;
			bytes /= 2;

			data.assoc_info.resp_ies = os_malloc(bytes);
			if (data.assoc_info.resp_ies == NULL)
				goto done;

			data.assoc_info.resp_ies_len = bytes;
			hexstr2bin(spos, data.assoc_info.resp_ies, bytes);
		}

		wpa_supplicant_event(ctx, EVENT_ASSOCINFO, &data);

	done:
		os_free(data.assoc_info.resp_ies);
		os_free(data.assoc_info.req_ies);
#ifdef CONFIG_PEERKEY
	} else if (os_strncmp(custom, "STKSTART.request=", 17) == 0) {
		if (hwaddr_aton(custom + 17, data.stkstart.peer)) {
			wpa_printf(MSG_DEBUG, "WEXT: unrecognized "
				   "STKSTART.request '%s'", custom + 17);
			return;
		}
		wpa_supplicant_event(ctx, EVENT_STKSTART, &data);
#endif /* CONFIG_PEERKEY */
	}
#ifdef TI_WLAN_WILINK_6_SUPPORT
	else
	{
		pData = (IPC_EV_DATA *)custom;
		switch (pData->EvParams.uEventType) {
			case	IPC_EVENT_ASSOCIATED:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_ASSOCIATED");
				break;
			case	IPC_EVENT_DISASSOCIATED:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_DISASSOCIATED");
				break;
			case	IPC_EVENT_LINK_SPEED:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_LINK_SPEED");
				if(pData->uBufferSize == sizeof(u32))
				{
					wpa_printf(MSG_DEBUG, "update link_speed");
					drv->link_speed = *((u32 *)pData->uBuffer) / 2;
				}
				
				wpa_printf(MSG_INFO,"wpa_supplicant - Link Speed = %u", drv->link_speed );
				break;
			case	IPC_EVENT_AUTH_SUCC:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_AUTH_SUCC");
				break;
			case	IPC_EVENT_SCAN_REPORT:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_SCAN_REPORT");
				break;
			case	IPC_EVENT_SCAN_COMPLETE:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_SCAN_COMPLETE");
				break;
			case	IPC_EVENT_TIMEOUT:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_TIMEOUT");
				break;
			case	IPC_EVENT_CCKM_START:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_CCKM_START");
				break;
			case	IPC_EVENT_MEDIA_SPECIFIC:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_MEDIA_SPECIFIC");
				break;
			case	IPC_EVENT_EAPOL:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_EAPOL");
				break;
			case	IPC_EVENT_BOUND:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_BOUND");
				break;
			case	IPC_EVENT_UNBOUND:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_UNBOUND");
				break;
			case	IPC_EVENT_PREAUTH_EAPOL:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_PREAUTH_EAPOL");
				break;
			case	IPC_EVENT_RESERVED2:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_RESERVED2");
				break;
			case	IPC_EVENT_LOW_RSSI:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_LOW_RSSI");
				break;
			case	IPC_EVENT_TSPEC_STATUS:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_TSPEC_STATUS");
				break;
			case	IPC_EVENT_TSPEC_RATE_STATUS:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_TSPEC_RATE_STATUS");
				break;
			case	IPC_EVENT_MEDIUM_TIME_CROSS:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_MEDIUM_TIME_CROSS");
				break;
			case	IPC_EVENT_ROAMING_COMPLETE:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_ROAMING_COMPLETE");
				break;
			case	IPC_EVENT_EAP_AUTH_FAILURE:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_EAP_AUTH_FAILURE");
				break;
			case	IPC_EVENT_WPA2_PREAUTHENTICATION:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_WPA2_PREAUTHENTICATION");
				break;
			case	IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED");
				break;
			case	IPC_EVENT_GWSI:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_GWSI");
				break;
			case	IPC_EVENT_WPS_SESSION_OVERLAP:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_WPS_SESSION_OVERLAP");
				break;
			case	IPC_EVENT_RSSI_SNR_TRIGGER_0:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_RSSI_SNR_TRIGGER_0");
				break;
			case	IPC_EVENT_RSSI_SNR_TRIGGER_1:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_RSSI_SNR_TRIGGER_1");
				break;
			case	IPC_EVENT_LOGGER:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_LOGGER");
				break;
			case	IPC_EVENT_NOT_ASSOCIATED:
				wpa_printf(MSG_DEBUG, "IPC_EVENT_NOT_ASSOCIATED");
				break;
			default:
				wpa_printf(MSG_DEBUG, "Unknown event");
				break;
		}
	}
#endif
}

static int wpa_driver_tista_event_wireless_michaelmicfailure(
	void *ctx, const char *ev, size_t len)
{
	const struct iw_michaelmicfailure *mic;
	union wpa_event_data data;

	if (len < sizeof(*mic))
		return -1;

	mic = (const struct iw_michaelmicfailure *) ev;

	wpa_printf(MSG_DEBUG, "Michael MIC failure wireless event: "
		   "flags=0x%x src_addr=" MACSTR, mic->flags,
		   MAC2STR(mic->src_addr.sa_data));

	os_memset(&data, 0, sizeof(data));
	data.michael_mic_failure.unicast = !(mic->flags & IW_MICFAILURE_GROUP);
	wpa_supplicant_event(ctx, EVENT_MICHAEL_MIC_FAILURE, &data);

	return 0;
}

static int wpa_driver_tista_event_wireless_pmkidcand(
	struct wpa_driver_tista_data *drv, const char *ev, size_t len)
{
	const struct iw_pmkid_cand *cand;
	union wpa_event_data data;
	const u8 *addr;

	if (len < sizeof(*cand))
		return -1;

	cand = (const struct iw_pmkid_cand *) ev;
	addr = (const u8 *) cand->bssid.sa_data;

	wpa_printf(MSG_DEBUG, "PMKID candidate wireless event: "
		   "flags=0x%x index=%d bssid=" MACSTR, cand->flags,
		   cand->index, MAC2STR(addr));

	os_memset(&data, 0, sizeof(data));
	os_memcpy(data.pmkid_candidate.bssid, addr, ETH_ALEN);
	data.pmkid_candidate.index = cand->index;
	data.pmkid_candidate.preauth = cand->flags & IW_PMKID_CAND_PREAUTH;
	wpa_supplicant_event(drv->ctx, EVENT_PMKID_CANDIDATE, &data);

	return 0;
}

static int wpa_driver_tista_event_wireless_assocreqie(
	struct wpa_driver_tista_data *drv, const char *ev, int len)
{
	if (len < 0)
		return -1;

	wpa_hexdump(MSG_DEBUG, "AssocReq IE wireless event", (const u8 *) ev,
		    len);
	os_free(drv->assoc_req_ies);
	drv->assoc_req_ies = os_malloc(len);
	if (drv->assoc_req_ies == NULL) {
		drv->assoc_req_ies_len = 0;
		return -1;
	}
	os_memcpy(drv->assoc_req_ies, ev, len);
	drv->assoc_req_ies_len = len;

	return 0;
}

static int wpa_driver_tista_event_wireless_assocrespie(
	struct wpa_driver_tista_data *drv, const char *ev, int len)
{
	if (len < 0)
		return -1;

	wpa_hexdump(MSG_DEBUG, "AssocResp IE wireless event", (const u8 *) ev,
		    len);
	os_free(drv->assoc_resp_ies);
	drv->assoc_resp_ies = os_malloc(len);
	if (drv->assoc_resp_ies == NULL) {
		drv->assoc_resp_ies_len = 0;
		return -1;
	}
	os_memcpy(drv->assoc_resp_ies, ev, len);
	drv->assoc_resp_ies_len = len;

	return 0;
}

static void wpa_driver_tista_event_assoc_ies(struct wpa_driver_tista_data *drv)
{
	union wpa_event_data data;

	if (drv->assoc_req_ies == NULL && drv->assoc_resp_ies == NULL)
		return;

	os_memset(&data, 0, sizeof(data));
	if (drv->assoc_req_ies) {
		data.assoc_info.req_ies = drv->assoc_req_ies;
		drv->assoc_req_ies = NULL;
		data.assoc_info.req_ies_len = drv->assoc_req_ies_len;
	}
	if (drv->assoc_resp_ies) {
		data.assoc_info.resp_ies = drv->assoc_resp_ies;
		drv->assoc_resp_ies = NULL;
		data.assoc_info.resp_ies_len = drv->assoc_resp_ies_len;
	}

	wpa_supplicant_event(drv->ctx, EVENT_ASSOCINFO, &data);

	os_free(data.assoc_info.req_ies);
	os_free(data.assoc_info.resp_ies);
}

static void wpa_driver_tista_event_wireless(struct wpa_driver_tista_data *drv,
					   void *ctx, char *data, int len)
{
	struct iw_event iwe_buf, *iwe = &iwe_buf;
	char *pos, *end, *custom, *buf;

	pos = data;
	end = data + len;

	wpa_hexdump(MSG_MSGDUMP, "wext event", (unsigned char *)data, len);

	while (pos + IW_EV_LCP_LEN <= end) {
		/* Event data may be unaligned, so make a local, aligned copy
		 * before processing. */
		os_memcpy(&iwe_buf, pos, IW_EV_LCP_LEN);
		wpa_printf(MSG_DEBUG, "Wireless event: cmd=0x%x len=%d",
			   iwe->cmd, iwe->len);
		if (iwe->len <= IW_EV_LCP_LEN)
			return;

		custom = pos + IW_EV_POINT_LEN;
		if (drv->we_version_compiled > 18 &&
		    (iwe->cmd == IWEVMICHAELMICFAILURE ||
		     iwe->cmd == IWEVCUSTOM ||
		     iwe->cmd == IWEVASSOCREQIE ||
		     iwe->cmd == IWEVASSOCRESPIE ||
		     iwe->cmd == IWEVPMKIDCAND)) {
			/* WE-19 removed the pointer from struct iw_point */
			char *dpos = (char *) &iwe_buf.u.data.length;
			int dlen = dpos - (char *) &iwe_buf;
			os_memcpy(dpos, pos + IW_EV_LCP_LEN,
				  sizeof(struct iw_event) - dlen);
		} else {
			os_memcpy(&iwe_buf, pos, sizeof(struct iw_event));
			custom += IW_EV_POINT_OFF;
		}

		wpa_printf(MSG_DEBUG, "WEXT: wpa_driver_tista_event_wireless cmd 0x%x", iwe->cmd);

		switch (iwe->cmd) {
		case SIOCGIWAP:
			wpa_printf(MSG_DEBUG, "Wireless event: new AP: "
				   MACSTR,
				   MAC2STR((u8 *) iwe->u.ap_addr.sa_data));
			if (os_memcmp(iwe->u.ap_addr.sa_data,
				      "\x00\x00\x00\x00\x00\x00", ETH_ALEN) ==
			    0 ||
			    os_memcmp(iwe->u.ap_addr.sa_data,
				      "\x44\x44\x44\x44\x44\x44", ETH_ALEN) ==
			    0) {
				os_free(drv->assoc_req_ies);
				drv->assoc_req_ies = NULL;
				os_free(drv->assoc_resp_ies);
				drv->assoc_resp_ies = NULL;
				wpa_supplicant_event(ctx, EVENT_DISASSOC,
						     NULL);
			
			} else {
				wpa_driver_tista_event_assoc_ies(drv);
				wpa_supplicant_event(ctx, EVENT_ASSOC, NULL);
			}
			break;
		case IWEVMICHAELMICFAILURE:
			wpa_driver_tista_event_wireless_michaelmicfailure(
				ctx, custom, iwe->u.data.length);
			break;
		case IWEVCUSTOM:
			if (custom + iwe->u.data.length > end)
				return;
			buf = os_malloc(iwe->u.data.length + 1);
			if (buf == NULL)
				return;
			os_memcpy(buf, custom, iwe->u.data.length);
			buf[iwe->u.data.length] = '\0';
			wpa_driver_tista_event_wireless_custom(drv, ctx, buf);
			os_free(buf);
			break;
		case SIOCGIWSCAN:
			drv->scan_complete_events = 1;
			eloop_cancel_timeout(wpa_driver_tista_scan_timeout,
					     drv, ctx);
			wpa_supplicant_event(ctx, EVENT_SCAN_RESULTS, NULL);
			break;
		case IWEVASSOCREQIE:
			wpa_driver_tista_event_wireless_assocreqie(
				drv, custom, iwe->u.data.length);
			break;
		case IWEVASSOCRESPIE:
			wpa_driver_tista_event_wireless_assocrespie(
				drv, custom, iwe->u.data.length);
			break;
		case IWEVPMKIDCAND:
			wpa_driver_tista_event_wireless_pmkidcand(
				drv, custom, iwe->u.data.length);
			break;
		}

		pos += iwe->len;
	}
}

static void wpa_driver_tista_event_link(void *ctx, char *buf, size_t len,
				       int del)
{
	union wpa_event_data event;

	os_memset(&event, 0, sizeof(event));
	if (len > sizeof(event.interface_status.ifname))
		len = sizeof(event.interface_status.ifname) - 1;
	os_memcpy(event.interface_status.ifname, buf, len);
	event.interface_status.ievent = del ? EVENT_INTERFACE_REMOVED :
		EVENT_INTERFACE_ADDED;

	wpa_printf(MSG_DEBUG, "RTM_%sLINK, IFLA_IFNAME: Interface '%s' %s",
		   del ? "DEL" : "NEW",
		   event.interface_status.ifname,
		   del ? "removed" : "added");

	wpa_supplicant_event(ctx, EVENT_INTERFACE_STATUS, &event);
}

static void wpa_driver_tista_event_rtm_newlink(struct wpa_driver_tista_data *drv,
					      void *ctx, struct nlmsghdr *h,
					      size_t len)
{
	struct ifinfomsg *ifi;
	int attrlen, nlmsg_len, rta_len;
	struct rtattr * attr;

	if (len < sizeof(*ifi))
		return;

	ifi = NLMSG_DATA(h);

	if (drv->ifindex != ifi->ifi_index && drv->ifindex2 != ifi->ifi_index)
	{
		wpa_printf(MSG_DEBUG, "Ignore event for foreign ifindex %d",
			   ifi->ifi_index);
		return;
	}

	wpa_printf(MSG_DEBUG, "RTM_NEWLINK: operstate=%d ifi_flags=0x%x "
		   "(%s%s%s%s)",
		   drv->operstate, ifi->ifi_flags,
		   (ifi->ifi_flags & IFF_UP) ? "[UP]" : "",
		   (ifi->ifi_flags & IFF_RUNNING) ? "[RUNNING]" : "",
		   (ifi->ifi_flags & IFF_LOWER_UP) ? "[LOWER_UP]" : "",
		   (ifi->ifi_flags & IFF_DORMANT) ? "[DORMANT" : "");
	/*
	 * Some drivers send the association event before the operup event--in
	 * this case, lifting operstate in wpa_driver_tista_set_operstate()
	 * fails. This will hit us when wpa_supplicant does not need to do
	 * IEEE 802.1X authentication
	 */
	if (drv->operstate == 1 &&
	    (ifi->ifi_flags & (IFF_LOWER_UP | IFF_DORMANT)) == IFF_LOWER_UP &&
	    !(ifi->ifi_flags & IFF_RUNNING))
		wpa_driver_tista_send_oper_ifla(drv, -1, IF_OPER_UP);

	nlmsg_len = NLMSG_ALIGN(sizeof(struct ifinfomsg));

	attrlen = h->nlmsg_len - nlmsg_len;
	if (attrlen < 0)
		return;

	attr = (struct rtattr *) (((char *) ifi) + nlmsg_len);

	rta_len = RTA_ALIGN(sizeof(struct rtattr));
	while (RTA_OK(attr, attrlen)) {
		if (attr->rta_type == IFLA_WIRELESS) {
			wpa_driver_tista_event_wireless(
				drv, ctx, ((char *) attr) + rta_len,
				attr->rta_len - rta_len);
		} else if (attr->rta_type == IFLA_IFNAME) {
			wpa_driver_tista_event_link(ctx,
						   ((char *) attr) + rta_len,
						   attr->rta_len - rta_len, 0);
		}
		attr = RTA_NEXT(attr, attrlen);
	}
}

static void wpa_driver_tista_event_rtm_dellink(struct wpa_driver_tista_data *drv,
					      void *ctx, struct nlmsghdr *h,
					      size_t len)
{
	struct ifinfomsg *ifi;
	int attrlen, nlmsg_len, rta_len;
	struct rtattr * attr;

	if (len < sizeof(*ifi))
		return;

	ifi = NLMSG_DATA(h);

	nlmsg_len = NLMSG_ALIGN(sizeof(struct ifinfomsg));

	attrlen = h->nlmsg_len - nlmsg_len;
	if (attrlen < 0)
		return;

	attr = (struct rtattr *) (((char *) ifi) + nlmsg_len);

	rta_len = RTA_ALIGN(sizeof(struct rtattr));
	while (RTA_OK(attr, attrlen)) {
		if (attr->rta_type == IFLA_IFNAME) {
			wpa_driver_tista_event_link(ctx,
						   ((char *) attr) + rta_len,
						   attr->rta_len - rta_len, 1);
		}
		attr = RTA_NEXT(attr, attrlen);
	}
}

static void wpa_driver_tista_event_receive(int sock, void *eloop_ctx,
					  void *sock_ctx)
{
	char buf[8192];
	int left;
	struct sockaddr_nl from;
	socklen_t fromlen;
	struct nlmsghdr *h;
	int max_events = 10;

try_again:
	fromlen = sizeof(from);
	left = recvfrom(sock, buf, sizeof(buf), MSG_DONTWAIT,
			(struct sockaddr *) &from, &fromlen);
	if (left < 0) {
		if (errno != EINTR && errno != EAGAIN)
			perror("recvfrom(netlink)");
		return;
	}

	h = (struct nlmsghdr *) buf;
	while (left >= (int) sizeof(*h)) {
		int len, plen;

		len = h->nlmsg_len;
		plen = len - sizeof(*h);
		if (len > left || plen < 0) {
			wpa_printf(MSG_DEBUG, "Malformed netlink message: "
				   "len=%d left=%d plen=%d",
				   len, left, plen);
			break;
		}

		switch (h->nlmsg_type) {
		case RTM_NEWLINK:
			wpa_driver_tista_event_rtm_newlink(eloop_ctx, sock_ctx,
							  h, plen);
			break;
		case RTM_DELLINK:
			wpa_driver_tista_event_rtm_dellink(eloop_ctx, sock_ctx,
							  h, plen);
			break;
		}

		len = NLMSG_ALIGN(len);
		left -= len;
		h = (struct nlmsghdr *) ((char *) h + len);
	}

	if (left > 0) {
		wpa_printf(MSG_DEBUG, "%d extra bytes in the end of netlink "
			   "message", left);
	}

	if (--max_events > 0) {
		/*
		 * Try to receive all events in one eloop call in order to
		 * limit race condition on cases where AssocInfo event, Assoc
		 * event, and EAPOL frames are received more or less at the
		 * same time. We want to process the event messages first
		 * before starting EAPOL processing.
		 */
		goto try_again;
	}
}

static int wpa_driver_tista_get_ifflags_ifname(struct wpa_driver_tista_data *drv,
					      const char *ifname, int *flags)
{
	struct ifreq ifr;

	os_memset(&ifr, 0, sizeof(ifr));
	os_strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
	if (ioctl(drv->ioctl_sock, SIOCGIFFLAGS, (caddr_t) &ifr) < 0) {
		perror("ioctl[SIOCGIFFLAGS]");
		return -1;
	}
	*flags = ifr.ifr_flags & 0xffff;
	return 0;
}

/**
 * wpa_driver_tista_get_ifflags - Get interface flags (SIOCGIFFLAGS)
 * @drv: driver_tista private data
 * @flags: Pointer to returned flags value
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_tista_get_ifflags(struct wpa_driver_tista_data *drv, int *flags)
{
	return wpa_driver_tista_get_ifflags_ifname(drv, drv->ifname, flags);
}

static int wpa_driver_tista_set_ifflags_ifname(struct wpa_driver_tista_data *drv,
					      const char *ifname, int flags)
{
	struct ifreq ifr;

	os_memset(&ifr, 0, sizeof(ifr));
	os_strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
	ifr.ifr_flags = flags & 0xffff;
	if (ioctl(drv->ioctl_sock, SIOCSIFFLAGS, (caddr_t) &ifr) < 0) {
		perror("SIOCSIFFLAGS");
		return -1;
	}
	return 0;
}

/**
 * wpa_driver_tista_set_ifflags - Set interface flags (SIOCSIFFLAGS)
 * @drv: driver_tista private data
 * @flags: New value for flags
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_tista_set_ifflags(struct wpa_driver_tista_data *drv, int flags)
{
	return wpa_driver_tista_set_ifflags_ifname(drv, drv->ifname, flags);
}

/**
 * wpa_driver_tista_init - Initialize WE driver interface
 * @ctx: context to be used when calling wpa_supplicant functions,
 * e.g., wpa_supplicant_event()
 * @ifname: interface name, e.g., wlan0
 * Returns: Pointer to private data, %NULL on failure
 */
void * wpa_driver_tista_init(void *ctx, const char *ifname)
{
	int s, flags;
	struct sockaddr_nl local;
	struct wpa_driver_tista_data *drv;

	drv = os_zalloc(sizeof(*drv));
	if (drv == NULL)
		return NULL;
	drv->ctx = ctx;
	os_strncpy(drv->ifname, ifname, sizeof(drv->ifname));

	drv->ioctl_sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (drv->ioctl_sock < 0) {
		perror("socket(PF_INET,SOCK_DGRAM)");
		os_free(drv);
		return NULL;
	}

	s = socket(PF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
	if (s < 0) {
		perror("socket(PF_NETLINK,SOCK_RAW,NETLINK_ROUTE)");
		close(drv->ioctl_sock);
		os_free(drv);
		return NULL;
	}

	os_memset(&local, 0, sizeof(local));
	local.nl_family = AF_NETLINK;
	local.nl_groups = RTMGRP_LINK;
	if (bind(s, (struct sockaddr *) &local, sizeof(local)) < 0) {
		perror("bind(netlink)");
		close(s);
		close(drv->ioctl_sock);
		os_free(drv);
		return NULL;
	}

	eloop_register_read_sock(s, wpa_driver_tista_event_receive, drv, ctx);
	drv->event_sock = s;

	drv->mlme_sock = -1;

	/*
	 * Make sure that the driver does not have any obsolete PMKID entries.
	 */
	wpa_driver_tista_flush_pmkid(drv);

	if (wpa_driver_tista_set_mode(drv, 0) < 0) {
		printf("Could not configure driver to use managed mode\n");
	}

	if (wpa_driver_tista_get_ifflags(drv, &flags) != 0 ||
	    wpa_driver_tista_set_ifflags(drv, flags | IFF_UP) != 0) {
		printf("Could not set interface '%s' UP\n", drv->ifname);
	}

	wpa_driver_tista_get_range(drv);

	drv->ifindex = if_nametoindex(drv->ifname);

	if (os_strncmp(ifname, "wlan", 4) == 0) {
		/*
		 * Host AP driver may use both wlan# and wifi# interface in
		 * wireless events. Since some of the versions included WE-18
		 * support, let's add the alternative ifindex also from
		 * driver_tista.c for the time being. This may be removed at
		 * some point once it is believed that old versions of the
		 * driver are not in use anymore.
		 */
		char ifname2[IFNAMSIZ + 1];
		os_strncpy(ifname2, ifname, sizeof(ifname2));
		os_memcpy(ifname2, "wifi", 4);
		wpa_driver_tista_alternative_ifindex(drv, ifname2);
	}

	wpa_driver_tista_send_oper_ifla(drv, 1, IF_OPER_DORMANT);

#ifdef TI_WLAN_WILINK_6_SUPPORT
	/* Signal that driver is not loaded yet */
	drv->driver_is_loaded = TRUE;

	/* Set default scan type */
	drv->scan_type = SCAN_TYPE_NORMAL_ACTIVE;

	/* Set default amount of channels */
	drv->scan_channels = 14;

	/* Link Speed will be set by the message from the driver */
	drv->link_speed = 0;

	/* BtCoex mode is read from tiwlan.ini file */
	drv->btcoex_mode = 0; /* SG_DISABLE */
#endif

	return drv;
}

/**
 * wpa_driver_tista_deinit - Deinitialize WE driver interface
 * @priv: Pointer to private wext data from wpa_driver_tista_init()
 *
 * Shut down driver interface and processing of driver events. Free
 * private data buffer if one was allocated in wpa_driver_tista_init().
 */
void wpa_driver_tista_deinit(void *priv)
{
	struct wpa_driver_tista_data *drv = priv;
	int flags;

	eloop_cancel_timeout(wpa_driver_tista_scan_timeout, drv, drv->ctx);

	/*
	 * Clear possibly configured driver parameters in order to make it
	 * easier to use the driver after wpa_supplicant has been terminated.
	 */
	wpa_driver_tista_set_bssid(drv, (u8 *) "\x00\x00\x00\x00\x00\x00");

	wpa_driver_tista_send_oper_ifla(priv, 0, IF_OPER_UP);

	eloop_unregister_read_sock(drv->event_sock);
	if (drv->mlme_sock >= 0)
		eloop_unregister_read_sock(drv->mlme_sock);

	if (wpa_driver_tista_get_ifflags(drv, &flags) == 0)
		(void) wpa_driver_tista_set_ifflags(drv, flags & ~IFF_UP);

#ifdef CONFIG_CLIENT_MLME
	if (drv->mlmedev[0] &&
	    wpa_driver_tista_get_ifflags_ifname(drv, drv->mlmedev, &flags) == 0)
		(void) wpa_driver_tista_set_ifflags_ifname(drv, drv->mlmedev,
							  flags & ~IFF_UP);
#endif /* CONFIG_CLIENT_MLME */

	close(drv->event_sock);
	close(drv->ioctl_sock);
	if (drv->mlme_sock >= 0)
		close(drv->mlme_sock);
	os_free(drv->assoc_req_ies);
	os_free(drv->assoc_resp_ies);
	os_free(drv);
}

/**
 * wpa_driver_tista_scan_timeout - Scan timeout to report scan completion
 * @eloop_ctx: Unused
 * @timeout_ctx: ctx argument given to wpa_driver_tista_init()
 *
 * This function can be used as registered timeout when starting a scan to
 * generate a scan completed event if the driver does not report this.
 */
void wpa_driver_tista_scan_timeout(void *eloop_ctx, void *timeout_ctx)
{
	wpa_printf(MSG_DEBUG, "Scan timeout - try to get results");
	wpa_supplicant_event(timeout_ctx, EVENT_SCAN_RESULTS, NULL);
}

/* Compare function for sorting scan results. Return >0 if @b is considered
 * better. */
static int wpa_scan_result_compar(const void *a, const void *b)
{
	const struct wpa_scan_result *wa = a;
	const struct wpa_scan_result *wb = b;

	/* WPA/WPA2 support preferred */
	if ((wb->wpa_ie_len || wb->rsn_ie_len) &&
	    !(wa->wpa_ie_len || wa->rsn_ie_len))
		return 1;
	if (!(wb->wpa_ie_len || wb->rsn_ie_len) &&
	    (wa->wpa_ie_len || wa->rsn_ie_len))
		return -1;

	/* privacy support preferred */
	if ((wa->caps & IEEE80211_CAP_PRIVACY) == 0 &&
	    (wb->caps & IEEE80211_CAP_PRIVACY))
		return 1;
	if ((wa->caps & IEEE80211_CAP_PRIVACY) &&
	    (wb->caps & IEEE80211_CAP_PRIVACY) == 0)
		return -1;

	/* best/max rate preferred if signal level close enough XXX */
	if (wa->maxrate != wb->maxrate && abs(wb->level - wa->level) < 5)
		return wb->maxrate - wa->maxrate;

	/* use freq for channel preference */

	/* all things being equal, use signal level; if signal levels are
	 * identical, use quality values since some drivers may only report
	 * that value and leave the signal level zero */
	if (wb->level == wa->level)
		return wb->qual - wa->qual;
	return wb->level - wa->level;
}

/**
 * wpa_driver_tista_get_scan_results - Fetch the latest scan results
 * @priv: Pointer to private wext data from wpa_driver_tista_init()
 * @results: Pointer to buffer for scan results
 * @max_size: Maximum number of entries (buffer size)
 * Returns: Number of scan result entries used on success, -1 on
 * failure
 *
 * If scan results include more than max_size BSSes, max_size will be
 * returned and the remaining entries will not be included in the
 * buffer.
 */
int wpa_driver_tista_get_scan_results(void *priv,
				     struct wpa_scan_result *results,
				     size_t max_size)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iwreq iwr;
	size_t ap_num = 0;
	int first, maxrate;
	u8 *res_buf;
	struct iw_event iwe_buf, *iwe = &iwe_buf;
	char *pos, *end, *custom, *genie, *gpos, *gend;
	struct iw_param p;
	size_t len, clen, res_buf_len;

	os_memset(results, 0, max_size * sizeof(struct wpa_scan_result));

	wpa_printf(MSG_ERROR," +++++++++ SCAN RESULTS   +++++++ \n\n\n");
	wpa_printf(MSG_ERROR,"struct iw_event = %d, %d", sizeof(struct iw_event), IW_EV_LCP_LEN);
	res_buf_len = IW_SCAN_MAX_DATA;
	for (;;) {
		res_buf = os_malloc(res_buf_len);
		if (res_buf == NULL)
			return -1;
		os_memset(&iwr, 0, sizeof(iwr));
		os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
		iwr.u.data.pointer = res_buf;
		iwr.u.data.length = res_buf_len;

		if (ioctl(drv->ioctl_sock, SIOCGIWSCAN, &iwr) == 0)
			break;

		if (errno == E2BIG && res_buf_len < 100000) {
			os_free(res_buf);
			res_buf = NULL;
			res_buf_len *= 2;
			wpa_printf(MSG_DEBUG, "Scan results did not fit - "
				   "trying larger buffer (%lu bytes)",
				   (unsigned long) res_buf_len);
		} else {
			perror("ioctl[SIOCGIWSCAN]");
			os_free(res_buf);
			return -1;
		}
	}

	len = iwr.u.data.length;
	ap_num = 0;
	first = 1;

	pos = (char *) res_buf;
	end = (char *) res_buf + len;

	while (pos + IW_EV_LCP_LEN <= end) {
		int ssid_len;
		/* Event data may be unaligned, so make a local, aligned copy
		 * before processing. */
		os_memcpy(&iwe_buf, pos, IW_EV_LCP_LEN);
		wpa_printf(MSG_ERROR,"len = 0x%x, cmd = 0x%x, ver = %d", iwe->len, iwe->cmd, drv->we_version_compiled);
		if (iwe->len <= IW_EV_LCP_LEN)
			break;

		custom = pos + IW_EV_POINT_LEN;
		if (drv->we_version_compiled > 18 &&
		    (iwe->cmd == SIOCGIWESSID ||
		     iwe->cmd == SIOCGIWENCODE ||
		     iwe->cmd == IWEVGENIE ||
		     iwe->cmd == IWEVCUSTOM)) {
			/* WE-19 removed the pointer from struct iw_point */
			char *dpos = (char *) &iwe_buf.u.data.length;
			int dlen = dpos - (char *) &iwe_buf;
			os_memcpy(dpos, pos + IW_EV_LCP_LEN,
				  sizeof(struct iw_event) - dlen);
		} else {
			os_memcpy(&iwe_buf, pos, sizeof(struct iw_event));
			custom += IW_EV_POINT_OFF;
		}

		switch (iwe->cmd) {
		case SIOCGIWAP:
			if (!first)
				ap_num++;
			first = 0;
			if (ap_num < max_size) {
				os_memcpy(results[ap_num].bssid,
					  iwe->u.ap_addr.sa_data, ETH_ALEN);
			}
			break;
		case SIOCGIWMODE:
			if (ap_num >= max_size)
				break;
			if (iwe->u.mode == IW_MODE_ADHOC)
				results[ap_num].caps |= IEEE80211_CAP_IBSS;
			else if (iwe->u.mode == IW_MODE_MASTER ||
				 iwe->u.mode == IW_MODE_INFRA)
				results[ap_num].caps |= IEEE80211_CAP_ESS;
			break;
		case SIOCGIWESSID:
			ssid_len = iwe->u.essid.length;
			if (custom + ssid_len > end)
				break;
			if (iwe->u.essid.flags &&
			    ssid_len > 0 &&
			    ssid_len <= IW_ESSID_MAX_SIZE) {
				if (ap_num < max_size) {
					os_memcpy(results[ap_num].ssid, custom,
						  ssid_len);
					results[ap_num].ssid_len = ssid_len;
				}
			}
			break;
		case SIOCGIWFREQ:
			if (ap_num < max_size) {
				int divi = 1000000, i;
				if (iwe->u.freq.e == 0) {
					/*
					 * Some drivers do not report
					 * frequency, but a channel. Try to map
					 * this to frequency by assuming they
					 * are using IEEE 802.11b/g.
					 */
					if (iwe->u.freq.m >= 1 &&
					    iwe->u.freq.m <= 13) {
						results[ap_num].freq =
							2407 +
							5 * iwe->u.freq.m;
						break;
					} else if (iwe->u.freq.m == 14) {
						results[ap_num].freq = 2484;
						break;
					}
				}
				if (iwe->u.freq.e > 6) {
					wpa_printf(
						MSG_DEBUG, "Invalid freq "
						"in scan results (BSSID="
						MACSTR ": m=%d e=%d\n",
						MAC2STR(results[ap_num].bssid),
						iwe->u.freq.m, iwe->u.freq.e);
					break;
				}
				for (i = 0; i < iwe->u.freq.e; i++)
					divi /= 10;
				results[ap_num].freq = iwe->u.freq.m / divi;
			}
			wpa_printf(MSG_ERROR,"+++++++++ SIOCGIWFREQ  ++++++++++");
			break;
		case IWEVQUAL:
			if (ap_num < max_size) {
				results[ap_num].qual = iwe->u.qual.qual;
				results[ap_num].noise = (s8)iwe->u.qual.noise;
				results[ap_num].level = (s8)iwe->u.qual.level;
				wpa_printf(MSG_DEBUG, "qual %d noise %d level %d", iwe->u.qual.qual, iwe->u.qual.noise, iwe->u.qual.level);
			}
			wpa_printf(MSG_ERROR,"+++++++++ IWEVQUAL  ++++++++++");
			break;
		case SIOCGIWENCODE:
			if (ap_num < max_size &&
			    !(iwe->u.data.flags & IW_ENCODE_DISABLED))
				results[ap_num].caps |= IEEE80211_CAP_PRIVACY;
			break;
		case SIOCGIWRATE:
			custom = pos + IW_EV_LCP_LEN;
			clen = iwe->len;
			if (custom + clen > end)
				break;
			maxrate = 0;
			while (((ssize_t) clen) >=
			       (ssize_t) sizeof(struct iw_param)) {
				/* Note: may be misaligned, make a local,
				 * aligned copy */
				os_memcpy(&p, custom, sizeof(struct iw_param));
				if (p.value > maxrate)
					maxrate = p.value;
				clen -= sizeof(struct iw_param);
				custom += sizeof(struct iw_param);
			}
			if (ap_num < max_size)
				results[ap_num].maxrate = maxrate;
			break;
		case IWEVGENIE:
			if (ap_num >= max_size)
				break;
			gpos = genie = custom;
			gend = genie + iwe->u.data.length;
			if (gend > end) {
				wpa_printf(MSG_INFO, "IWEVGENIE overflow");
				break;
			}
			while (gpos + 1 < gend &&
			       gpos + 2 + (u8) gpos[1] <= gend) {
				u8 ie = gpos[0], ielen = gpos[1] + 2;
				if (ielen > SSID_MAX_WPA_IE_LEN) {
					gpos += ielen;
					continue;
				}
				switch (ie) {
				case GENERIC_INFO_ELEM:
					if (ielen < 2 + 4 ||
					    os_memcmp(&gpos[2],
						      "\x00\x50\xf2\x01", 4) !=
					    0)
						break;
					os_memcpy(results[ap_num].wpa_ie, gpos,
						  ielen);
					results[ap_num].wpa_ie_len = ielen;
					break;
				case RSN_INFO_ELEM:
					os_memcpy(results[ap_num].rsn_ie, gpos,
						  ielen);
					results[ap_num].rsn_ie_len = ielen;
					break;
				}
				gpos += ielen;
			}
			break;
		case IWEVCUSTOM:
			clen = iwe->u.data.length;
			if (custom + clen > end)
				break;
			if (clen > 7 &&
			    os_strncmp(custom, "wpa_ie=", 7) == 0 &&
			    ap_num < max_size) {
				char *spos;
				int bytes;
				spos = custom + 7;
				bytes = custom + clen - spos;
				if (bytes & 1)
					break;
				bytes /= 2;
				if (bytes > SSID_MAX_WPA_IE_LEN) {
					wpa_printf(MSG_INFO, "Too long WPA IE "
						   "(%d)", bytes);
					break;
				}
				hexstr2bin(spos, results[ap_num].wpa_ie,
					   bytes);
				results[ap_num].wpa_ie_len = bytes;
			} else if (clen > 7 &&
				   os_strncmp(custom, "rsn_ie=", 7) == 0 &&
				   ap_num < max_size) {
				char *spos;
				int bytes;
				spos = custom + 7;
				bytes = custom + clen - spos;
				if (bytes & 1)
					break;
				bytes /= 2;
				if (bytes > SSID_MAX_WPA_IE_LEN) {
					wpa_printf(MSG_INFO, "Too long RSN IE "
						   "(%d)", bytes);
					break;
				}
				hexstr2bin(spos, results[ap_num].rsn_ie,
					   bytes);
				results[ap_num].rsn_ie_len = bytes;
			}
			break;
		}

		pos += iwe->len;
	}
	os_free(res_buf);
	res_buf = NULL;
	if (!first)
		ap_num++;
	if (ap_num > max_size) {
		wpa_printf(MSG_DEBUG, "Too small scan result buffer - "
			   "%lu BSSes but room only for %lu",
			   (unsigned long) ap_num,
			   (unsigned long) max_size);
		ap_num = max_size;
	}
	qsort(results, ap_num, sizeof(struct wpa_scan_result),
	      wpa_scan_result_compar);

	wpa_printf(MSG_DEBUG, "Received %lu bytes of scan results (%lu BSSes)",
		   (unsigned long) len, (unsigned long) ap_num);

	return ap_num;
}

static int wpa_driver_tista_get_range(void *priv)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iw_range *range;
	struct iwreq iwr;
	int minlen;
	size_t buflen;

	/*
	 * Use larger buffer than struct iw_range in order to allow the
	 * structure to grow in the future.
	 */
	buflen = sizeof(struct iw_range) + 500;
	range = os_zalloc(buflen);
	if (range == NULL)
		return -1;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	iwr.u.data.pointer = (caddr_t) range;
	iwr.u.data.length = buflen;

	minlen = ((char *) &range->enc_capa) - (char *) range +
		sizeof(range->enc_capa);
	wpa_printf(MSG_DEBUG, "SIOCGIWRANGE: %d", sizeof(struct iw_range));
	if (ioctl(drv->ioctl_sock, SIOCGIWRANGE, &iwr) < 0) {
		perror("ioctl[SIOCGIWRANGE]");
		os_free(range);
		return -1;
	} else if (iwr.u.data.length >= minlen &&
		   range->we_version_compiled >= 18) {
		wpa_printf(MSG_DEBUG, "SIOCGIWRANGE: WE(compiled)=%d "
			   "WE(source)=%d enc_capa=0x%x",
			   range->we_version_compiled,
			   range->we_version_source,
			   range->enc_capa);
		drv->has_capability = 1;
		drv->we_version_compiled = range->we_version_compiled;
		if (range->enc_capa & IW_ENC_CAPA_WPA) {
			drv->capa.key_mgmt |= WPA_DRIVER_CAPA_KEY_MGMT_WPA |
				WPA_DRIVER_CAPA_KEY_MGMT_WPA_PSK;
		}
		if (range->enc_capa & IW_ENC_CAPA_WPA2) {
			drv->capa.key_mgmt |= WPA_DRIVER_CAPA_KEY_MGMT_WPA2 |
				WPA_DRIVER_CAPA_KEY_MGMT_WPA2_PSK;
		}
		drv->capa.enc |= WPA_DRIVER_CAPA_ENC_WEP40 |
			WPA_DRIVER_CAPA_ENC_WEP104;
		if (range->enc_capa & IW_ENC_CAPA_CIPHER_TKIP)
			drv->capa.enc |= WPA_DRIVER_CAPA_ENC_TKIP;
		if (range->enc_capa & IW_ENC_CAPA_CIPHER_CCMP)
			drv->capa.enc |= WPA_DRIVER_CAPA_ENC_CCMP;
		wpa_printf(MSG_DEBUG, "  capabilities: key_mgmt 0x%x enc 0x%x",
			   drv->capa.key_mgmt, drv->capa.enc);
	} else {
		wpa_printf(MSG_DEBUG, "SIOCGIWRANGE: %d %d %d", sizeof(struct iw_range), range->we_version_compiled, iwr.u.data.length);
		wpa_printf(MSG_DEBUG, "SIOCGIWRANGE: too old (short) data - "
			   "assuming WPA is not supported");
	}

	os_free(range);
	return 0;
}

static int wpa_driver_tista_set_wpa(void *priv, int enabled)
{
	struct wpa_driver_tista_data *drv = priv;
	wpa_printf(MSG_DEBUG, "%s", __FUNCTION__);

	return wpa_driver_tista_set_auth_param(drv, IW_AUTH_WPA_ENABLED,
					      enabled);
}

static int wpa_driver_tista_set_key_ext(void *priv, wpa_alg alg,
				       const u8 *addr, int key_idx,
				       int set_tx, const u8 *seq,
				       size_t seq_len,
				       const u8 *key, size_t key_len)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iwreq iwr;
	int ret = 0;
	struct iw_encode_ext *ext;

	if (seq_len > IW_ENCODE_SEQ_MAX_SIZE) {
		wpa_printf(MSG_DEBUG, "%s: Invalid seq_len %lu",
			   __FUNCTION__, (unsigned long) seq_len);
		return -1;
	}

	ext = os_zalloc(sizeof(*ext) + key_len);
	if (ext == NULL)
		return -1;
	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	iwr.u.encoding.flags = key_idx + 1;
	if (alg == WPA_ALG_NONE)
		iwr.u.encoding.flags |= IW_ENCODE_DISABLED;
	iwr.u.encoding.pointer = (caddr_t) ext;
	iwr.u.encoding.length = sizeof(*ext) + key_len;

	if (addr == NULL ||
	    os_memcmp(addr, "\xff\xff\xff\xff\xff\xff", ETH_ALEN) == 0)
		ext->ext_flags |= IW_ENCODE_EXT_GROUP_KEY;
	if (set_tx)
		ext->ext_flags |= IW_ENCODE_EXT_SET_TX_KEY;

	ext->addr.sa_family = ARPHRD_ETHER;
	if (addr)
		os_memcpy(ext->addr.sa_data, addr, ETH_ALEN);
	else
		os_memset(ext->addr.sa_data, 0xff, ETH_ALEN);
	if (key && key_len) {
		os_memcpy(ext + 1, key, key_len);
		ext->key_len = key_len;
	}
	switch (alg) {
	case WPA_ALG_NONE:
		ext->alg = IW_ENCODE_ALG_NONE;
		break;
	case WPA_ALG_WEP:
		ext->alg = IW_ENCODE_ALG_WEP;
		break;
	case WPA_ALG_TKIP:
		ext->alg = IW_ENCODE_ALG_TKIP;
		break;
	case WPA_ALG_CCMP:
		ext->alg = IW_ENCODE_ALG_CCMP;
		break;
	default:
		wpa_printf(MSG_DEBUG, "%s: Unknown algorithm %d",
			   __FUNCTION__, alg);
		os_free(ext);
		return -1;
	}

	if (seq && seq_len) {
		ext->ext_flags |= IW_ENCODE_EXT_RX_SEQ_VALID;
		os_memcpy(ext->rx_seq, seq, seq_len);
	}

	if (ioctl(drv->ioctl_sock, SIOCSIWENCODEEXT, &iwr) < 0) {
		ret = errno == EOPNOTSUPP ? -2 : -1;
		if (errno == ENODEV) {
			/*
			 * ndiswrapper seems to be returning incorrect error
			 * code.. */
			ret = -2;
		}

		perror("ioctl[SIOCSIWENCODEEXT]");
	}

	os_free(ext);
	return ret;
}

/**
 * wpa_driver_tista_set_key - Configure encryption key
 * @priv: Pointer to private wext data from wpa_driver_tista_init()
 * @priv: Private driver interface data
 * @alg: Encryption algorithm (%WPA_ALG_NONE, %WPA_ALG_WEP,
 *	%WPA_ALG_TKIP, %WPA_ALG_CCMP); %WPA_ALG_NONE clears the key.
 * @addr: Address of the peer STA or ff:ff:ff:ff:ff:ff for
 *	broadcast/default keys
 * @key_idx: key index (0..3), usually 0 for unicast keys
 * @set_tx: Configure this key as the default Tx key (only used when
 *	driver does not support separate unicast/individual key
 * @seq: Sequence number/packet number, seq_len octets, the next
 *	packet number to be used for in replay protection; configured
 *	for Rx keys (in most cases, this is only used with broadcast
 *	keys and set to zero for unicast keys)
 * @seq_len: Length of the seq, depends on the algorithm:
 *	TKIP: 6 octets, CCMP: 6 octets
 * @key: Key buffer; TKIP: 16-byte temporal key, 8-byte Tx Mic key,
 *	8-byte Rx Mic Key
 * @key_len: Length of the key buffer in octets (WEP: 5 or 13,
 *	TKIP: 32, CCMP: 16)
 * Returns: 0 on success, -1 on failure
 *
 * This function uses SIOCSIWENCODEEXT by default, but tries to use
 * SIOCSIWENCODE if the extended ioctl fails when configuring a WEP key.
 */
int wpa_driver_tista_set_key(void *priv, wpa_alg alg,
			    const u8 *addr, int key_idx,
			    int set_tx, const u8 *seq, size_t seq_len,
			    const u8 *key, size_t key_len)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iwreq iwr;
	int ret = 0;

	wpa_printf(MSG_DEBUG, "%s: alg=%d key_idx=%d set_tx=%d seq_len=%lu "
		   "key_len=%lu",
		   __FUNCTION__, alg, key_idx, set_tx,
		   (unsigned long) seq_len, (unsigned long) key_len);

	ret = wpa_driver_tista_set_key_ext(drv, alg, addr, key_idx, set_tx,
					  seq, seq_len, key, key_len);
	if (ret == 0)
		return 0;

	if (ret == -2 &&
	    (alg == WPA_ALG_NONE || alg == WPA_ALG_WEP)) {
		wpa_printf(MSG_DEBUG, "Driver did not support "
			   "SIOCSIWENCODEEXT, trying SIOCSIWENCODE");
		ret = 0;
	} else {
		wpa_printf(MSG_DEBUG, "Driver did not support "
			   "SIOCSIWENCODEEXT");
		return ret;
	}

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	iwr.u.encoding.flags = key_idx + 1;
	if (alg == WPA_ALG_NONE)
		iwr.u.encoding.flags |= IW_ENCODE_DISABLED;
	iwr.u.encoding.pointer = (caddr_t) key;
	iwr.u.encoding.length = key_len;

	if (ioctl(drv->ioctl_sock, SIOCSIWENCODE, &iwr) < 0) {
		perror("ioctl[SIOCSIWENCODE]");
		ret = -1;
	}

	if (set_tx && alg != WPA_ALG_NONE) {
		os_memset(&iwr, 0, sizeof(iwr));
		os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
		iwr.u.encoding.flags = key_idx + 1;
		iwr.u.encoding.pointer = (caddr_t) NULL;
		iwr.u.encoding.length = 0;
		if (ioctl(drv->ioctl_sock, SIOCSIWENCODE, &iwr) < 0) {
			perror("ioctl[SIOCSIWENCODE] (set_tx)");
			ret = -1;
		}
	}

	return ret;
}

static int wpa_driver_tista_set_countermeasures(void *priv,
					       int enabled)
{
	struct wpa_driver_tista_data *drv = priv;
	wpa_printf(MSG_DEBUG, "%s", __FUNCTION__);
	return wpa_driver_tista_set_auth_param(drv,
					      IW_AUTH_TKIP_COUNTERMEASURES,
					      enabled);
}

static int wpa_driver_tista_set_drop_unencrypted(void *priv,
						int enabled)
{
	struct wpa_driver_tista_data *drv = priv;
	wpa_printf(MSG_DEBUG, "%s", __FUNCTION__);
	drv->use_crypt = enabled;
	return wpa_driver_tista_set_auth_param(drv, IW_AUTH_DROP_UNENCRYPTED,
					      enabled);
}

static int wpa_driver_tista_mlme(struct wpa_driver_tista_data *drv,
				const u8 *addr, int cmd, int reason_code)
{
	struct iwreq iwr;
	struct iw_mlme mlme;
	int ret = 0;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	os_memset(&mlme, 0, sizeof(mlme));
	mlme.cmd = cmd;
	mlme.reason_code = reason_code;
	mlme.addr.sa_family = ARPHRD_ETHER;
	os_memcpy(mlme.addr.sa_data, addr, ETH_ALEN);
	iwr.u.data.pointer = (caddr_t) &mlme;
	iwr.u.data.length = sizeof(mlme);

	if (ioctl(drv->ioctl_sock, SIOCSIWMLME, &iwr) < 0) {
		perror("ioctl[SIOCSIWMLME]");
		ret = -1;
	}

	return ret;
}

static int wpa_driver_tista_deauthenticate(void *priv, const u8 *addr,
					  int reason_code)
{
	struct wpa_driver_tista_data *drv = priv;
	wpa_printf(MSG_DEBUG, "%s", __FUNCTION__);
	return wpa_driver_tista_mlme(drv, addr, IW_MLME_DEAUTH, reason_code);
}

static int wpa_driver_tista_disassociate(void *priv, const u8 *addr,
					int reason_code)
{
	struct wpa_driver_tista_data *drv = priv;
	wpa_printf(MSG_DEBUG, "%s", __FUNCTION__);
	return wpa_driver_tista_mlme(drv, addr, IW_MLME_DISASSOC,
				    reason_code);
}

#ifndef TI_WLAN_WILINK_6_SUPPORT
static int wpa_driver_tista_set_gen_ie(void *priv, const u8 *ie,
				      size_t ie_len)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iwreq iwr;
	int ret = 0;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	iwr.u.data.pointer = (caddr_t) ie;
	iwr.u.data.length = ie_len;

	if (ioctl(drv->ioctl_sock, SIOCSIWGENIE, &iwr) < 0) {
		perror("ioctl[SIOCSIWGENIE]");
		ret = -1;
	}

	return ret;
}
#endif

static int wpa_driver_tista_cipher2wext(int cipher)
{
	switch (cipher) {
	case CIPHER_NONE:
		return IW_AUTH_CIPHER_NONE;
	case CIPHER_WEP40:
		return IW_AUTH_CIPHER_WEP40;
	case CIPHER_TKIP:
		return IW_AUTH_CIPHER_TKIP;
	case CIPHER_CCMP:
		return IW_AUTH_CIPHER_CCMP;
	case CIPHER_WEP104:
		return IW_AUTH_CIPHER_WEP104;
	default:
		return 0;
	}
}

static int wpa_driver_tista_keymgmt2wext(int keymgmt)
{
	switch (keymgmt) {
	case KEY_MGMT_802_1X:
	case KEY_MGMT_802_1X_NO_WPA:
		return IW_AUTH_KEY_MGMT_802_1X;
	case KEY_MGMT_PSK:
		return IW_AUTH_KEY_MGMT_PSK;
	default:
		return 0;
	}
}

static int
wpa_driver_tista_auth_alg_fallback(struct wpa_driver_tista_data *drv,
				  struct wpa_driver_associate_params *params)
{
	struct iwreq iwr;
	int ret = 0;

	wpa_printf(MSG_DEBUG, "WEXT: Driver did not support "
		   "SIOCSIWAUTH for AUTH_ALG, trying SIOCSIWENCODE");

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	/* Just changing mode, not actual keys */
	iwr.u.encoding.flags = 0;
	iwr.u.encoding.pointer = (caddr_t) NULL;
	iwr.u.encoding.length = 0;

	/*
	 * Note: IW_ENCODE_{OPEN,RESTRICTED} can be interpreted to mean two
	 * different things. Here they are used to indicate Open System vs.
	 * Shared Key authentication algorithm. However, some drivers may use
	 * them to select between open/restricted WEP encrypted (open = allow
	 * both unencrypted and encrypted frames; restricted = only allow
	 * encrypted frames).
	 */

	if (!drv->use_crypt) {
		iwr.u.encoding.flags |= IW_ENCODE_DISABLED;
	} else {
		if (params->auth_alg & AUTH_ALG_OPEN_SYSTEM)
			iwr.u.encoding.flags |= IW_ENCODE_OPEN;
		if (params->auth_alg & AUTH_ALG_SHARED_KEY)
			iwr.u.encoding.flags |= IW_ENCODE_RESTRICTED;
	}

	if (ioctl(drv->ioctl_sock, SIOCSIWENCODE, &iwr) < 0) {
		perror("ioctl[SIOCSIWENCODE]");
		ret = -1;
	}

	return ret;
}

static int
wpa_driver_tista_associate(void *priv,
			  struct wpa_driver_associate_params *params)
{
	struct wpa_driver_tista_data *drv = priv;
	int ret = 0;
	int allow_unencrypted_eapol;
	int value;

	wpa_printf(MSG_DEBUG, "%s", __FUNCTION__);

	/*
	 * If the driver did not support SIOCSIWAUTH, fallback to
	 * SIOCSIWENCODE here.
	 */
	if (drv->auth_alg_fallback &&
	    wpa_driver_tista_auth_alg_fallback(drv, params) < 0)
		ret = -1;

	if (!params->bssid &&
	    wpa_driver_tista_set_bssid(drv, NULL) < 0)
		ret = -1;

	if (wpa_driver_tista_set_mode(drv, params->mode) < 0)
		ret = -1;
	/* TODO: should consider getting wpa version and cipher/key_mgmt suites
	 * from configuration, not from here, where only the selected suite is
	 * available */
#ifndef TI_WLAN_WILINK_6_SUPPORT
	if (wpa_driver_tista_set_gen_ie(drv, params->wpa_ie, params->wpa_ie_len)
	    < 0)
		ret = -1;
#endif
	if (params->wpa_ie == NULL || params->wpa_ie_len == 0)
		value = IW_AUTH_WPA_VERSION_DISABLED;
	else if (params->wpa_ie[0] == RSN_INFO_ELEM)
		value = IW_AUTH_WPA_VERSION_WPA2;
	else
		value = IW_AUTH_WPA_VERSION_WPA;
	if (wpa_driver_tista_set_auth_param(drv,
					   IW_AUTH_WPA_VERSION, value) < 0)
		ret = -1;
	value = wpa_driver_tista_cipher2wext(params->pairwise_suite);
	if (wpa_driver_tista_set_auth_param(drv,
					   IW_AUTH_CIPHER_PAIRWISE, value) < 0)
		ret = -1;
	value = wpa_driver_tista_cipher2wext(params->group_suite);
	if (wpa_driver_tista_set_auth_param(drv,
					   IW_AUTH_CIPHER_GROUP, value) < 0)
		ret = -1;
	value = wpa_driver_tista_keymgmt2wext(params->key_mgmt_suite);
	if (wpa_driver_tista_set_auth_param(drv,
					   IW_AUTH_KEY_MGMT, value) < 0)
		ret = -1;
	value = params->key_mgmt_suite != KEY_MGMT_NONE ||
		params->pairwise_suite != CIPHER_NONE ||
		params->group_suite != CIPHER_NONE ||
		params->wpa_ie_len;
	if (wpa_driver_tista_set_auth_param(drv,
					   IW_AUTH_PRIVACY_INVOKED, value) < 0)
		ret = -1;

	/* Allow unencrypted EAPOL messages even if pairwise keys are set when
	 * not using WPA. IEEE 802.1X specifies that these frames are not
	 * encrypted, but WPA encrypts them when pairwise keys are in use. */
	if (params->key_mgmt_suite == KEY_MGMT_802_1X ||
	    params->key_mgmt_suite == KEY_MGMT_PSK)
		allow_unencrypted_eapol = 0;
	else
		allow_unencrypted_eapol = 1;
	
	if (wpa_driver_tista_set_auth_param(drv,
					   IW_AUTH_RX_UNENCRYPTED_EAPOL,
					   allow_unencrypted_eapol) < 0)
		ret = -1;
	if (params->freq && wpa_driver_tista_set_freq(drv, params->freq) < 0)
		ret = -1;
	if (wpa_driver_tista_set_ssid(drv, params->ssid, params->ssid_len) < 0)
		ret = -1;
	if (params->bssid &&
	    wpa_driver_tista_set_bssid(drv, params->bssid) < 0)
		ret = -1;

	return ret;
}

static int wpa_driver_tista_set_auth_alg(void *priv, int auth_alg)
{
	struct wpa_driver_tista_data *drv = priv;
	int algs = 0, res;

	if (auth_alg & AUTH_ALG_OPEN_SYSTEM)
		algs |= IW_AUTH_ALG_OPEN_SYSTEM;
	if (auth_alg & AUTH_ALG_SHARED_KEY)
		algs |= IW_AUTH_ALG_SHARED_KEY;
	if (auth_alg & AUTH_ALG_LEAP)
		algs |= IW_AUTH_ALG_LEAP;
	if (algs == 0) {
		/* at least one algorithm should be set */
		algs = IW_AUTH_ALG_OPEN_SYSTEM;
	}

	res = wpa_driver_tista_set_auth_param(drv, IW_AUTH_80211_AUTH_ALG,
					     algs);
	drv->auth_alg_fallback = res == -2;
	return res;
}

/**
 * wpa_driver_tista_set_mode - Set wireless mode (infra/adhoc), SIOCSIWMODE
 * @priv: Pointer to private wext data from wpa_driver_tista_init()
 * @mode: 0 = infra/BSS (associate with an AP), 1 = adhoc/IBSS
 * Returns: 0 on success, -1 on failure
 */
int wpa_driver_tista_set_mode(void *priv, int mode)
{
	struct wpa_driver_tista_data *drv = priv;
	struct iwreq iwr;
	int ret = 0;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	iwr.u.mode = mode ? IW_MODE_ADHOC : IW_MODE_INFRA;

	if (ioctl(drv->ioctl_sock, SIOCSIWMODE, &iwr) < 0) {
		perror("ioctl[SIOCSIWMODE]");
		ret = -1;
	}

	return ret;
}

static int wpa_driver_tista_pmksa(struct wpa_driver_tista_data *drv,
				 u32 cmd, const u8 *bssid, const u8 *pmkid)
{
	struct iwreq iwr;
	struct iw_pmksa pmksa;
	int ret = 0;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	os_memset(&pmksa, 0, sizeof(pmksa));
	pmksa.cmd = cmd;
	pmksa.bssid.sa_family = ARPHRD_ETHER;
	if (bssid)
		os_memcpy(pmksa.bssid.sa_data, bssid, ETH_ALEN);
	if (pmkid)
		os_memcpy(pmksa.pmkid, pmkid, IW_PMKID_LEN);
	iwr.u.data.pointer = (caddr_t) &pmksa;
	iwr.u.data.length = sizeof(pmksa);

	if (ioctl(drv->ioctl_sock, SIOCSIWPMKSA, &iwr) < 0) {
		if (errno != EOPNOTSUPP)
			perror("ioctl[SIOCSIWPMKSA]");
		ret = -1;
	}

	return ret;
}

static int wpa_driver_tista_add_pmkid(void *priv, const u8 *bssid,
				     const u8 *pmkid)
{
	struct wpa_driver_tista_data *drv = priv;
	return wpa_driver_tista_pmksa(drv, IW_PMKSA_ADD, bssid, pmkid);
}

static int wpa_driver_tista_remove_pmkid(void *priv, const u8 *bssid,
		 			const u8 *pmkid)
{
	struct wpa_driver_tista_data *drv = priv;
	return wpa_driver_tista_pmksa(drv, IW_PMKSA_REMOVE, bssid, pmkid);
}


static int wpa_driver_tista_flush_pmkid(void *priv)
{
	struct wpa_driver_tista_data *drv = priv;
	return wpa_driver_tista_pmksa(drv, IW_PMKSA_FLUSH, NULL, NULL);
}

static int wpa_driver_tista_get_capa(void *priv, struct wpa_driver_capa *capa)
{
	struct wpa_driver_tista_data *drv = priv;
	if (!drv->has_capability)
		return -1;
	os_memcpy(capa, &drv->capa, sizeof(*capa));
	return 0;
}

int wpa_driver_tista_alternative_ifindex(struct wpa_driver_tista_data *drv,
					const char *ifname)
{
	if (ifname == NULL) {
		drv->ifindex2 = -1;
		return 0;
	}

	drv->ifindex2 = if_nametoindex(ifname);
	if (drv->ifindex2 <= 0)
		return -1;

	wpa_printf(MSG_DEBUG, "Added alternative ifindex %d (%s) for "
		   "wireless events", drv->ifindex2, ifname);

	return 0;
}

int wpa_driver_tista_set_operstate(void *priv, int state)
{
	struct wpa_driver_tista_data *drv = priv;

	wpa_printf(MSG_DEBUG, "%s: operstate %d->%d (%s)",
		   __func__, drv->operstate, state, state ? "UP" : "DORMANT");
	drv->operstate = state;
	return wpa_driver_tista_send_oper_ifla(
		drv, -1, state ? IF_OPER_UP : IF_OPER_DORMANT);
}


#ifdef CONFIG_CLIENT_MLME
static int hostapd_ioctl(struct wpa_driver_tista_data *drv,
			 struct prism2_hostapd_param *param, int len)
{
	struct iwreq iwr;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	iwr.u.data.pointer = (caddr_t) param;
	iwr.u.data.length = len;

	if (ioctl(drv->ioctl_sock, PRISM2_IOCTL_HOSTAPD, &iwr) < 0) {
		perror("ioctl[PRISM2_IOCTL_HOSTAPD]");
		return -1;
	}

	return 0;
}

static struct wpa_hw_modes *
wpa_driver_tista_get_hw_feature_data(void *priv, u16 *num_modes, u16 *flags)
{
	struct wpa_driver_tista_data *drv = priv;
	struct prism2_hostapd_param *param;
	u8 *pos, *end;
	struct wpa_hw_modes *modes = NULL;
	int i;

	param = os_zalloc(PRISM2_HOSTAPD_MAX_BUF_SIZE);
	if (param == NULL)
		return NULL;
	param->cmd = PRISM2_HOSTAPD_GET_HW_FEATURES;

	if (hostapd_ioctl(drv, param, PRISM2_HOSTAPD_MAX_BUF_SIZE) < 0) {
		perror("ioctl[PRISM2_IOCTL_HOSTAPD]");
		goto out;
	}

	*num_modes = param->u.hw_features.num_modes;
	*flags = param->u.hw_features.flags;

	pos = param->u.hw_features.data;
	end = pos + PRISM2_HOSTAPD_MAX_BUF_SIZE -
		(param->u.hw_features.data - (u8 *) param);

	modes = os_zalloc(*num_modes * sizeof(struct wpa_hw_modes));
	if (modes == NULL)
		goto out;

	for (i = 0; i < *num_modes; i++) {
		struct hostapd_ioctl_hw_modes_hdr *hdr;
		struct wpa_hw_modes *feature;
		int clen, rlen;

		hdr = (struct hostapd_ioctl_hw_modes_hdr *) pos;
		pos = (u8 *) (hdr + 1);
		clen = hdr->num_channels * sizeof(struct wpa_channel_data);
		rlen = hdr->num_rates * sizeof(struct wpa_rate_data);

		feature = &modes[i];
		switch (hdr->mode) {
		case MODE_IEEE80211A:
			feature->mode = WPA_MODE_IEEE80211A;
			break;
		case MODE_IEEE80211B:
			feature->mode = WPA_MODE_IEEE80211B;
			break;
		case MODE_IEEE80211G:
			feature->mode = WPA_MODE_IEEE80211G;
			break;
		case MODE_ATHEROS_TURBO:
		case MODE_ATHEROS_TURBOG:
			wpa_printf(MSG_ERROR, "Skip unsupported hw_mode=%d in "
				   "get_hw_features data", hdr->mode);
			pos += clen + rlen;
			continue;
		default:
			wpa_printf(MSG_ERROR, "Unknown hw_mode=%d in "
				   "get_hw_features data", hdr->mode);
			ieee80211_sta_free_hw_features(modes, *num_modes);
			modes = NULL;
			break;
		}
		feature->num_channels = hdr->num_channels;
		feature->num_rates = hdr->num_rates;

		feature->channels = os_malloc(clen);
		feature->rates = os_malloc(rlen);
		if (!feature->channels || !feature->rates ||
		    pos + clen + rlen > end) {
			ieee80211_sta_free_hw_features(modes, *num_modes);
			modes = NULL;
			break;
		}

		os_memcpy(feature->channels, pos, clen);
		pos += clen;
		os_memcpy(feature->rates, pos, rlen);
		pos += rlen;
	}

out:
	os_free(param);
	return modes;
}

int wpa_driver_tista_set_channel(void *priv, wpa_hw_mode phymode, int chan,
				int freq)
{
	return wpa_driver_tista_set_freq(priv, freq);
}

static void wpa_driver_tista_mlme_read(int sock, void *eloop_ctx,
				      void *sock_ctx)
{
	struct wpa_driver_tista_data *drv = eloop_ctx;
	int len;
	unsigned char buf[3000];
	struct ieee80211_frame_info *fi;
	struct ieee80211_rx_status rx_status;

	len = recv(sock, buf, sizeof(buf), 0);
	if (len < 0) {
		perror("recv[MLME]");
		return;
	}

	if (len < (int) sizeof(struct ieee80211_frame_info)) {
		wpa_printf(MSG_DEBUG, "WEXT: Too short MLME frame (len=%d)",
			   len);
		return;
	}

	fi = (struct ieee80211_frame_info *) buf;
	if (ntohl(fi->version) != IEEE80211_FI_VERSION) {
		wpa_printf(MSG_DEBUG, "WEXT: Invalid MLME frame info version "
			   "0x%x", ntohl(fi->version));
		return;
	}

	os_memset(&rx_status, 0, sizeof(rx_status));
	rx_status.ssi = ntohl(fi->ssi_signal);
	rx_status.channel = ntohl(fi->channel);

	ieee80211_sta_rx(drv->ctx, buf + sizeof(struct ieee80211_frame_info),
			 len - sizeof(struct ieee80211_frame_info),
			 &rx_status);
}

static int wpa_driver_tista_open_mlme(struct wpa_driver_tista_data *drv)
{
	int flags, ifindex, s, *i;
	struct sockaddr_ll addr;
	struct iwreq iwr;

	os_memset(&iwr, 0, sizeof(iwr));
	os_strncpy(iwr.ifr_name, drv->ifname, IFNAMSIZ);
	i = (int *) iwr.u.name;
	*i++ = PRISM2_PARAM_USER_SPACE_MLME;
	*i++ = 1;

	if (ioctl(drv->ioctl_sock, PRISM2_IOCTL_PRISM2_PARAM, &iwr) < 0) {
		wpa_printf(MSG_ERROR, "WEXT: Failed to configure driver to "
			   "use user space MLME");
		return -1;
	}

	ifindex = if_nametoindex(drv->mlmedev);
	if (ifindex == 0) {
		wpa_printf(MSG_ERROR, "WEXT: mlmedev='%s' not found",
			   drv->mlmedev);
		return -1;
	}

	if (wpa_driver_tista_get_ifflags_ifname(drv, drv->mlmedev, &flags) != 0
	    || wpa_driver_tista_set_ifflags_ifname(drv, drv->mlmedev,
						  flags | IFF_UP) != 0) {
		wpa_printf(MSG_ERROR, "WEXT: Could not set interface "
			   "'%s' UP", drv->mlmedev);
		return -1;
	}

	s = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (s < 0) {
		perror("socket[PF_PACKET,SOCK_RAW]");
		return -1;
	}

	os_memset(&addr, 0, sizeof(addr));
	addr.sll_family = AF_PACKET;
	addr.sll_ifindex = ifindex;

	if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		perror("bind(MLME)");
		return -1;
	}

	if (eloop_register_read_sock(s, wpa_driver_tista_mlme_read, drv, NULL))
	{
		wpa_printf(MSG_ERROR, "WEXT: Could not register MLME read "
			   "socket");
		close(s);
		return -1;
	}

	return s;
}

static int wpa_driver_tista_send_mlme(void *priv, const u8 *data,
				     size_t data_len)
{
	struct wpa_driver_tista_data *drv = priv;
	int ret;

	ret = send(drv->mlme_sock, data, data_len, 0);
	if (ret < 0) {
		perror("send[MLME]");
		return -1;
	}

	return 0;
}

static int wpa_driver_tista_mlme_add_sta(void *priv, const u8 *addr,
					const u8 *supp_rates,
					size_t supp_rates_len)
{
	struct wpa_driver_tista_data *drv = priv;
	struct prism2_hostapd_param param;
	size_t len;

	os_memset(&param, 0, sizeof(param));
	param.cmd = PRISM2_HOSTAPD_ADD_STA;
	os_memcpy(param.sta_addr, addr, ETH_ALEN);
	len = supp_rates_len;
	if (len > sizeof(param.u.add_sta.supp_rates))
		len = sizeof(param.u.add_sta.supp_rates);
	os_memcpy(param.u.add_sta.supp_rates, supp_rates, len);
	return hostapd_ioctl(drv, &param, sizeof(param));
}

static int wpa_driver_tista_mlme_remove_sta(void *priv, const u8 *addr)
{
	struct wpa_driver_tista_data *drv = priv;
	struct prism2_hostapd_param param;

	os_memset(&param, 0, sizeof(param));
	param.cmd = PRISM2_HOSTAPD_REMOVE_STA;
	os_memcpy(param.sta_addr, addr, ETH_ALEN);
	return hostapd_ioctl(drv, &param, sizeof(param));
}

#endif /* CONFIG_CLIENT_MLME */

static int wpa_driver_tista_set_param(void *priv, const char *param)
{
#ifdef CONFIG_CLIENT_MLME
	struct wpa_driver_tista_data *drv = priv;
	const char *pos, *pos2;
	size_t len;

	if (param == NULL)
		return 0;

	wpa_printf(MSG_DEBUG, "%s: param='%s'", __func__, param);

	pos = os_strstr(param, "mlmedev=");
	if (pos) {
		pos += 8;
		pos2 = os_strchr(pos, ' ');
		if (pos2)
			len = pos2 - pos;
		else
			len = os_strlen(pos);
		if (len + 1 > sizeof(drv->mlmedev))
			return -1;
		os_memcpy(drv->mlmedev, pos, len);
		drv->mlmedev[len] = '\0';
		wpa_printf(MSG_DEBUG, "WEXT: Using user space MLME with "
			   "mlmedev='%s'", drv->mlmedev);
		drv->capa.flags |= WPA_DRIVER_FLAGS_USER_SPACE_MLME;

		drv->mlme_sock = wpa_driver_tista_open_mlme(drv);
		if (drv->mlme_sock < 0)
			return -1;
	}
#endif /* CONFIG_CLIENT_MLME */

	return 0;
}

int wpa_driver_tista_get_version(struct wpa_driver_tista_data *drv)
{
	return drv->we_version_compiled;
}

const struct wpa_driver_ops wpa_driver_custom_ops = {
	.name = TIWLAN_DRV_NAME,
	.desc = "TI Station Driver (1271)",
	.get_bssid = wpa_driver_tista_get_bssid,
	.get_ssid = wpa_driver_tista_get_ssid,
	.set_wpa = wpa_driver_tista_set_wpa,
	.set_key = wpa_driver_tista_set_key,
	.set_countermeasures = wpa_driver_tista_set_countermeasures,
	.set_drop_unencrypted = wpa_driver_tista_set_drop_unencrypted,
	.scan = wpa_driver_tista_scan,
	.get_scan_results = wpa_driver_tista_get_scan_results,
	.deauthenticate = wpa_driver_tista_deauthenticate,
	.disassociate = wpa_driver_tista_disassociate,
	.associate = wpa_driver_tista_associate,
	.set_auth_alg = wpa_driver_tista_set_auth_alg,
	.init = wpa_driver_tista_init,
	.deinit = wpa_driver_tista_deinit,
	.set_param = wpa_driver_tista_set_param,
	.add_pmkid = wpa_driver_tista_add_pmkid,
	.remove_pmkid = wpa_driver_tista_remove_pmkid,
	.flush_pmkid = wpa_driver_tista_flush_pmkid,
	.get_capa = wpa_driver_tista_get_capa,
	.set_operstate = wpa_driver_tista_set_operstate,
#ifdef CONFIG_CLIENT_MLME
	.get_hw_feature_data = wpa_driver_tista_get_hw_feature_data,
	.set_channel = wpa_driver_tista_set_channel,
	.set_ssid = wpa_driver_tista_set_ssid,
	.set_bssid = wpa_driver_tista_set_bssid,
	.send_mlme = wpa_driver_tista_send_mlme,
	.mlme_add_sta = wpa_driver_tista_mlme_add_sta,
	.mlme_remove_sta = wpa_driver_tista_mlme_remove_sta,
#endif /* CONFIG_CLIENT_MLME */
#ifdef TI_WLAN_WILINK_6_SUPPORT
	.driver_cmd = wpa_driver_tista_driver_cmd
#endif
};
