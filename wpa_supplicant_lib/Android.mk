LOCAL_PATH := $(call my-dir)
WPA_BUILD_SUPPLICANT_LIB := false
ifeq ($(HAVE_CUSTOM_WIFI_DRIVER_2),true)
WPA_BUILD_SUPPLICANT_LIB := true
STA_DK_ROOT = $(LOCAL_PATH)/../..
endif

DK_ROOT = $(STA_DK_ROOT)
OS_ROOT = $(STA_DK_ROOT)/pform
COMMON  = $(DK_ROOT)/common
COMSRC  = $(COMMON)/src
CUDK_ROOT = $(DK_ROOT)/CUDK

include external/wpa_supplicant/.config

# To force sizeof(enum) = 4
ifneq ($(TARGET_SIMULATOR),true)
L_CFLAGS += -mabi=aapcs-linux
endif

INCLUDES = $(COMMON)/inc \
	$(COMSRC)/inc \
	$(COMSRC)/utils \
	$(COMSRC)/hal/inc \
	$(COMSRC)/hal/TnetwServices  \
	$(COMSRC)/hal/TnetwServices/TNETW1150  \
	$(COMSRC)/hal/FirmwareApi \
	$(COMSRC)/hal/hl_data \
	$(COMSRC)/hal/hl_ctrl \
	$(COMSRC)/hal/hl_ctrl/PowerCtrl \
	$(COMSRC)/hal/hw_data \
	$(COMSRC)/hal/hw_ctrl \
	$(COMSRC)/hal/security \
	$(COMSRC)/hal/security/CCX \
	$(COMSRC)/hal/security/Privacy \
	$(COMSRC)/hal/security/Privacy/Enc/Ckip \
	$(COMSRC)/hal/security/Privacy/Enc/KP \
	$(COMSRC)/hal/security/Privacy/Enc/KP/Ckip \
	$(COMSRC)/hal/security/Privacy/ICV \
	$(COMSRC)/hal/security/Privacy/IV \
	$(COMSRC)/hal/security/Privacy/IV/WEP \
	$(COMSRC)/hal/security/Privacy/MIC \
	$(COMSRC)/hal/security/Privacy/MIC/Michael \
	$(COMSRC)/hal/security/Privacy/MIC/MMH \
	$(COMSRC)/hal/security/Privacy/Seq \
	$(COMSRC)/hal/security/Privacy/Seq/CCX \
	$(COMSRC)/hal/Export_Inc \
	$(COMSRC)/BusAccess/Shm_Common \
	$(COMSRC)/BusAccess/Export_Inc \
	$(COMSRC)/BusAccess/Shm_Slave \
	$(COMSRC)/core/inc \
	$(COMSRC)/core/ExpInc \
	$(COMSRC)/core/data_ctrl/Tx \
	$(COMSRC)/core/data_ctrl/Ctrl \
	$(COMSRC)/core/data_ctrl/Ctrl/TrafficMonitor \
	$(COMSRC)/core/data_ctrl/Ctrl/4X \
	$(COMSRC)/core/data_ctrl/Rx \
	$(COMSRC)/core/EvHandler \
	$(COMSRC)/core/sme/Inc \
	$(COMSRC)/core/sme/smeSm/ \
	$(COMSRC)/core/sme/siteMgr \
	$(COMSRC)/core/sme/configMgr \
	$(COMSRC)/core/sme/conn \
	$(COMSRC)/core/sme/HealthMonitor \
	$(COMSRC)/core/srv/inc \
	$(COMSRC)/core/srv/scanSrv \
	$(COMSRC)/core/srv/scr \
	$(COMSRC)/core/rsn \
	$(COMSRC)/core/rsn/mainKeysSm \
	$(COMSRC)/core/rsn/mainKeysSm/keyDerive \
	$(COMSRC)/core/rsn/adm_ctrl/WPA \
	$(COMSRC)/core/rsn/adm_ctrl/WPA2 \
	$(COMSRC)/core/rsn/adm_ctrl/WEP/CCX \
	$(COMSRC)/core/rsn/inc \
	$(COMSRC)/core/rsn/algorithms \
	$(COMSRC)/core/mlme \
	$(COMSRC)/core/mlme/Assoc \
	$(COMSRC)/core/mlme/Auth/open \
	$(COMSRC)/core/mlme/Auth/shared \
	$(COMSRC)/Management/QOS/Inc/ \
	$(COMSRC)/Management/CCX/Inc/ \
	$(COMSRC)/Management/Roaming/Inc/ \
	$(COMSRC)/core/NetworkCtrl/inc \
	$(COMSRC)/Management/AirLink/Measurement \
	$(COMSRC)/Management/AirLink/Measurement/CCX \
	$(COMSRC)/Management/AirLink/Measurement/dot11h \
	$(COMSRC)/Management/AirLink/inc \
	$(COMSRC)/Management/scan/inc \
	$(COMSRC)/Management/scan/scanCncn \
	$(COMSRC)/core/NetworkCtrl/RegulatoryDomain \
	$(COMSRC)/core/NetworkCtrl/RegulatoryDomain/CCX \
	$(COMSRC)/Management/PowerMgr/ \
	$(COMSRC)/core/NetworkCtrl/QOS \
	$(COMSRC)/Application/inc \
	$(COMSRC)/Application/\ExpInc \
	$(COMSRC)/Application/ScanMngr \
	$(COMSRC)/Management/apConn \
	$(COMSRC)/core/currBss \
	$(COMSRC)/Management/scan/inc \
	$(CUDK_ROOT)/Inc \
	$(OS_ROOT)/common/inc \
	$(OS_ROOT)/linux/inc \
	$(OS_ROOT)/linux/src \
	$(COMSRC)/core/EvHandler \
	$(CUDK_ROOT)/IPC/Linux \
	$(CUDK_ROOT)/UtilityAdapter \
	external/openssl/include \
	external/wpa_supplicant
  
L_CFLAGS += -DCONFIG_DRIVER_CUSTOM -DHOST_COMPILE
ifeq ($(HAVE_CUSTOM_WIFI_DRIVER_0),true) 
L_CFLAGS += -DSTA_DK_VER_5_0_0_94 
endif
OBJS = driver_ti.c

ifdef CONFIG_NO_STDOUT_DEBUG
L_CFLAGS += -DCONFIG_NO_STDOUT_DEBUG
endif

ifdef CONFIG_DEBUG_FILE
L_CFLAGS += -DCONFIG_DEBUG_FILE
endif

ifdef CONFIG_IEEE8021X_EAPOL
L_CFLAGS += -DIEEE8021X_EAPOL
endif

ifneq ($(TARGET_SIMULATOR),true)
ifeq ($(WPA_BUILD_SUPPLICANT_LIB),true)

########################

include $(CLEAR_VARS)
LOCAL_MODULE := libCustomWifi
LOCAL_STATIC_LIBRARIES := libWifiApi
LOCAL_SHARED_LIBRARIES := libc
LOCAL_CFLAGS := $(L_CFLAGS)
LOCAL_SRC_FILES := $(OBJS)
LOCAL_C_INCLUDES := $(INCLUDES)
include $(BUILD_STATIC_LIBRARY)

########################

endif # ifeq ($(WPA_BUILD_SUPPLICANT_LIB),true)
endif # ifneq ($(TARGET_SIMULATOR),true)
