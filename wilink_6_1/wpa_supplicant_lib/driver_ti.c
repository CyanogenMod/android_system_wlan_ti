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

#define TIWLAN_DRV_NAME         "tiwlan0"

typedef enum {
    BLUETOOTH_COEXISTENCE_MODE_ENABLED = 0,
    BLUETOOTH_COEXISTENCE_MODE_DISABLED,
    BLUETOOTH_COEXISTENCE_MODE_SENSE
} EUIBTCoexMode;

struct wpa_driver_tista_data {
	void *wext; /* private data for driver_wext */
	void *ctx;
	char ifname[IFNAMSIZ + 1];
	int ioctl_sock;
#ifdef TI_WLAN_WILINK_6_SUPPORT
	u8 own_addr[ETH_ALEN];          /* MAC address of WLAN interface */
	int driver_is_loaded;           /* TRUE/FALSE flag if driver is already loaded and can be accessed */
	int scan_type;                  /* SCAN_TYPE_NORMAL_ACTIVE or  SCAN_TYPE_NORMAL_PASSIVE */
	int scan_channels;              /* Number of allowed scan channels */
	unsigned int link_speed;        /* Link Speed */
	u32 btcoex_mode;                /* BtCoex Mode */
#endif
};

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

static int wpa_driver_tista_get_bssid(void *priv, u8 *bssid)
{
	struct wpa_driver_tista_data *drv = priv;
	return wpa_driver_wext_get_bssid(drv->wext, bssid);
}

static int wpa_driver_tista_get_ssid(void *priv, u8 *ssid)
{
	struct wpa_driver_tista_data *drv = priv;
	return wpa_driver_wext_get_ssid(drv->wext, ssid);
}

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
	    wpa_printf(MSG_ERROR, "ERROR - wpa_driver_tista_private_send - error sending Wext private IOCTL to STA driver (ioctl_cmd = %x,  res = %d, errno = %d)", ioctl_cmd, res, errno);
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

int wpa_driver_tista_parse_custom(void *ctx, const void *custom)
{
	IPC_EV_DATA * pData = NULL;

	pData = (IPC_EV_DATA *)custom;
	wpa_printf(MSG_DEBUG, "uEventType %d", pData->EvParams.uEventType);
	switch (pData->EvParams.uEventType) {
		case	IPC_EVENT_LINK_SPEED:
			wpa_printf(MSG_DEBUG, "IPC_EVENT_LINK_SPEED");
			if(pData->uBufferSize == sizeof(u32))
			{
				wpa_printf(MSG_DEBUG, "update link_speed");
				/* Dm: pStaDrv->link_speed = *((u32 *)pData->uBuffer) / 2; */
			}

			/* Dm: wpa_printf(MSG_INFO,"wpa_supplicant - Link Speed = %u", pStaDrv->link_speed ); */
			break;
		default:
			wpa_printf(MSG_DEBUG, "Unknown event");
			break;
	}

	return 0;
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

	wpa_printf(MSG_DEBUG, "%s", __func__);
	return wpa_driver_wext_scan(drv->wext, ssid, ssid_len);
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

	if(is_set) /* set power mode */
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
	else /* get power mode */
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


/**
 * wpa_driver_tista_init - Initialize WE driver interface
 * @ctx: context to be used when calling wpa_supplicant functions,
 * e.g., wpa_supplicant_event()
 * @ifname: interface name, e.g., wlan0
 * Returns: Pointer to private data, %NULL on failure
 */
void * wpa_driver_tista_init(void *ctx, const char *ifname)
{
	struct wpa_driver_tista_data *drv;

	drv = os_zalloc(sizeof(*drv));
	if (drv == NULL)
		return NULL;
	drv->wext = wpa_driver_wext_init(ctx, ifname);
	if (drv->wext == NULL) {
		os_free(drv);
		return NULL;
	}

	drv->ctx = ctx;
	os_strncpy(drv->ifname, ifname, sizeof(drv->ifname));
	drv->ioctl_sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (drv->ioctl_sock < 0) {
		perror("socket");
		wpa_driver_wext_deinit(drv->wext);
		os_free(drv);
		return NULL;
	}

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

	wpa_driver_wext_deinit(drv->wext);
	close(drv->ioctl_sock);
	os_free(drv);
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

static int wpa_driver_tista_set_wpa(void *priv, int enabled)
{
	struct wpa_driver_tista_data *drv = priv;
	return wpa_driver_tista_set_auth_param(drv, IW_AUTH_WPA_ENABLED,
					      enabled);
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

	return res;
}

static int wpa_driver_tista_set_countermeasures(void *priv, int enabled)
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
	/* Dm: drv->use_crypt = enabled; */
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

static int wpa_driver_tista_set_key(void *priv, wpa_alg alg,
			    const u8 *addr, int key_idx,
			    int set_tx, const u8 *seq, size_t seq_len,
			    const u8 *key, size_t key_len)
{
	struct wpa_driver_tista_data *drv = priv;
	wpa_printf(MSG_DEBUG, "%s", __func__);
	return wpa_driver_wext_set_key(drv->wext, alg, addr, key_idx, set_tx,
		                                                 seq, seq_len, key, key_len);
}

/*-----------------------------------------------------------------------------
Compare function for sorting scan results. Return >0 if @b is considered better.
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_scan_result_compare(const void *a, const void *b)
{
    const struct wpa_scan_result *wa = a;
    const struct wpa_scan_result *wb = b;

    return( wb->level - wa->level );
}

static int wpa_driver_tista_get_scan_results(void *priv,
					      struct wpa_scan_result *results,
					      size_t max_size)
{
	struct wpa_driver_tista_data *drv = priv;
	size_t ap_num = 0;
	
	ap_num = wpa_driver_wext_get_scan_results(drv->wext, results, max_size);
	wpa_printf(MSG_DEBUG, "Actual APs number %d", ap_num);
	/* Dm: ap_num = scan_merge( drv, results, ap_num, max_size );
	wpa_printf(MSG_DEBUG, "After merge, APs number %d", ap_num);
	qsort( results, ap_num, sizeof(struct wpa_scan_result),
           wpa_driver_tista_scan_result_compare ); */
	return ap_num;
}

static int wpa_driver_tista_associate(void *priv,
			  struct wpa_driver_associate_params *params)
{
	struct wpa_driver_tista_data *drv = priv;
	int allow_unencrypted_eapol;
	int value;

	/* Set driver network mode (Adhoc/Infrastructure) according to supplied parameters */
	wpa_driver_wext_set_mode( drv->wext, params->mode);

	if (params->wpa_ie == NULL || params->wpa_ie_len == 0)
		value = IW_AUTH_WPA_VERSION_DISABLED;
	else if (params->wpa_ie[0] == RSN_INFO_ELEM)
		value = IW_AUTH_WPA_VERSION_WPA2;
	else
		value = IW_AUTH_WPA_VERSION_WPA;
	wpa_driver_tista_set_auth_param(drv, IW_AUTH_WPA_VERSION, value);
	value = wpa_driver_tista_cipher2wext(params->pairwise_suite);
	wpa_driver_tista_set_auth_param(drv, IW_AUTH_CIPHER_PAIRWISE, value);
	value = wpa_driver_tista_cipher2wext(params->group_suite);
	wpa_driver_tista_set_auth_param(drv, IW_AUTH_CIPHER_GROUP, value);
	value = wpa_driver_tista_keymgmt2wext(params->key_mgmt_suite);
	wpa_driver_tista_set_auth_param(drv, IW_AUTH_KEY_MGMT, value);
	value = params->key_mgmt_suite != KEY_MGMT_NONE ||
		params->pairwise_suite != CIPHER_NONE ||
		params->group_suite != CIPHER_NONE ||
		params->wpa_ie_len;
	wpa_driver_tista_set_auth_param(drv, IW_AUTH_PRIVACY_INVOKED, value);

	/* Allow unencrypted EAPOL messages even if pairwise keys are set when
	 * not using WPA. IEEE 802.1X specifies that these frames are not
	 * encrypted, but WPA encrypts them when pairwise keys are in use. */
	if (params->key_mgmt_suite == KEY_MGMT_802_1X ||
	    params->key_mgmt_suite == KEY_MGMT_PSK)
		allow_unencrypted_eapol = 0;
	else
		allow_unencrypted_eapol = 1;
	
	wpa_driver_tista_set_auth_param(drv,
					   IW_AUTH_RX_UNENCRYPTED_EAPOL,
					   allow_unencrypted_eapol);

	if( params->bssid ) {
		wpa_printf(MSG_DEBUG, "wpa_driver_tista_associate: BSSID=" MACSTR, 
			            MAC2STR(params->bssid));
		/* if there is bssid -> set it */
		if( os_memcmp( params->bssid, "\x00\x00\x00\x00\x00\x00", ETH_ALEN ) != 0 ) {
			wpa_driver_wext_set_bssid( drv->wext, params->bssid );
		}
	}

	return wpa_driver_wext_set_ssid(drv->wext, params->ssid, params->ssid_len);
}

static int wpa_driver_tista_set_operstate(void *priv, int state)
{
	struct wpa_driver_tista_data *drv = priv;

	wpa_printf(MSG_DEBUG, "%s: operstate %d (%s)",
		   __func__, /*drv->operstate,*/ state, state ? "UP" : "DORMANT");
	/* Dm: drv->operstate = state; */
	return wpa_driver_wext_set_operstate(drv->wext, state);
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
	.get_mac_addr = wpa_driver_tista_get_mac_addr,
	.init = wpa_driver_tista_init,
	.deinit = wpa_driver_tista_deinit,
	.set_operstate = wpa_driver_tista_set_operstate,
	.driver_cmd = wpa_driver_tista_driver_cmd
};
