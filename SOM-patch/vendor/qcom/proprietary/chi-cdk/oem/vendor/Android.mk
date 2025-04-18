ifneq ($(CAMX_CHICDK_PATH),)
LOCAL_PATH := $(CAMX_CHICDK_PATH)/oem
else
LOCAL_PATH := $(call my-dir)
endif

#include $(CAMX_CHICDK_OEM_PATH)/qcom/eeprom/pmd_irs2381c_eeprom/build/android/Android.mk
#include $(CAMX_CHICDK_OEM_PATH)/qcom/eeprom/truly_cmb433_eeprom/build/android/Android.mk

ifeq ($(TARGET_BOARD_PLATFORM),lito)
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2anchorsync/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2demux/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2frameselect/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2fusion/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2generic/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2graphselector/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2hdr/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2memcpy/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2mfsr/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2qcfa/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2rawhdr/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2rt/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2serializer/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2stub/bitra/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2swmf/bitra/build/android/Android.mk
endif

include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2anchorsync/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2demux/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2frameselect/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2fusion/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2generic/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2graphselector/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2hdr/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2memcpy/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2mfsr/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2qcfa/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2rawhdr/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2rt/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2serializer/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2stub/common/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/feature2/chifeature2swmf/common/build/android/Android.mk

include $(CAMX_CHICDK_OEM_PATH)/qcom/node/aecwrapper/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/afwrapper/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/awbwrapper/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/customhwnode/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/depth/build/android/Android.mk

ifeq ($(TARGET_BOARD_PLATFORM),$(MSMSTEPPE))
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/dewarp/build/android/Android.mk
endif

include $(CAMX_CHICDK_OEM_PATH)/qcom/node/dummyrtb/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/dummysat/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/dummystich/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/fcv/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/gpu/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/hafoverride/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/hvx/addconstant/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/hvx/binning/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/memcpy/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/ai-director/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/nodeutils/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/remosaic/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/lt6911uxc/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/c2d/build/android/Android.mk

ifeq ($(TARGET_BOARD_PLATFORM),$(MSMSTEPPE))
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/swcac/build/android/Android.mk
endif

include $(CAMX_CHICDK_OEM_PATH)/qcom/node/staticaecalgo/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/staticafalgo/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/staticawbalgo/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/node/staticpdlibalgo/build/android/Android.mk
include $(CAMX_CHICDK_OEM_PATH)/qcom/sensor/imx577/build/android/Android.mk
###
# ACHEUL
###
include $(CAMX_CHICDK_OEM_PATH)/qcom/sensor/imx471/build/android/Android.mk


ifeq ($(TARGET_BOARD_PLATFORM),kona)
endif

include $(CAMX_CHICDK_OEM_PATH)/qcom/utils/chilog/build/android/Android.mk
