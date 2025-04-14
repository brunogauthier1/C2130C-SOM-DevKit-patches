#!/bin/bash

SOURCE_DIR="/media/bruno/T7/devkit-1.1-FC/LINUX/android/kernel"

SOURCE_FILES+="msm-4.19/techpack/display/msm/dp/dp_display.c"
SOURCE_FILES+=" msm-4.19/techpack/display/msm/sde/sde_kms.c"
SOURCE_FILES+=" msm-4.19/techpack/display/msm/sde/sde_wb.c"
SOURCE_FILES+=" msm-4.19/techpack/display/msm/sde/sde_rm.c"
SOURCE_FILES+=" msm-4.19/techpack/display/msm/dsi/dsi_display.c"
SOURCE_FILES+=" msm-4.19/techpack/display/msm/dsi/dsi_panel.c"
SOURCE_FILES+=" msm-4.19/techpack/display/msm/dsi/dsi_ctrl.c"
SOURCE_FILES+=" msm-4.19/techpack/display/msm/dsi/dsi_pwr.c"
SOURCE_FILES+=" msm-4.19/techpack/camera/drivers/cam_sensor_module/cam_cci/cam_cci_core.c"
SOURCE_FILES+=" msm-4.19/techpack/camera/drivers/cam_sensor_module/cam_sensor_utils/cam_sensor_util.c"
SOURCE_FILES+=" msm-4.19/techpack/camera/drivers/cam_sensor_module/cam_sensor/cam_sensor_core.c"
SOURCE_FILES+=" msm-4.19/techpack/camera/drivers/cam_sensor_module/cam_sensor_io/cam_sensor_io.c"
SOURCE_FILES+=" msm-4.19/techpack/camera/drivers/cam_sensor_module/cam_res_mgr/cam_res_mgr.c"
SOURCE_FILES+=" msm-4.19/arch/arm64/configs/vendor/kona_defconfig"
SOURCE_FILES+=" msm-4.19/drivers/gpu/drm/omapdrm/dss/dsi.c"
SOURCE_FILES+=" msm-4.19/drivers/gpu/drm/drm_print.c"
SOURCE_FILES+=" msm-4.19/drivers/iio/industrialio-core.c"
SOURCE_FILES+=" msm-4.19/drivers/iio/imu/inv_icm42600/inv_icm42600_accel.c"
SOURCE_FILES+=" msm-4.19/drivers/iio/imu/inv_icm42600/inv_icm42600_i2c.c"
SOURCE_FILES+=" msm-4.19/drivers/iio/imu/inv_icm42600/inv_icm42600_core.c"
SOURCE_FILES+=" msm-4.19/drivers/iio/imu/inv_icm42600/inv_icm42600_gyro.c"
SOURCE_FILES+=" msm-4.19/drivers/iio/imu/Kconfig"
SOURCE_FILES+=" msm-4.19/drivers/iio/imu/Makefile"
SOURCE_FILES+=" msm-4.19/drivers/iio/light/vcnl4000.c"
SOURCE_FILES+=" msm-4.19/drivers/iio/light/Kconfig"
SOURCE_FILES+=" msm-4.19/drivers/iio/proximity/Kconfig"
SOURCE_FILES+=" msm-4.19/drivers/iio/proximity/vcnl3040.c"
SOURCE_FILES+=" msm-4.19/drivers/iio/proximity/Makefile"
SOURCE_FILES+=" msm-4.19/drivers/regulator/devres.c"
SOURCE_FILES+=" msm-4.19/drivers/regulator/core.c"
SOURCE_FILES+=" msm-4.19/drivers/soc/qcom/msm_bus/msm_bus_of.c"
SOURCE_FILES+=" msm-4.19/drivers/input/misc/akm09911.c"
SOURCE_FILES+=" msm-4.19/drivers/input/misc/Makefile"
SOURCE_FILES+=" msm-4.19/drivers/input/Makefile"
SOURCE_FILES+=" msm-4.19/include/linux/akm09911.h"

DEST_DIR="./kernel"
NB_FILES=0

for current_file in ${SOURCE_FILES} ;
do
    file_path="$(echo ${current_file} | sed 's|\(.*\)\/.*|\1|g')"
    file_name="$(echo ${current_file} | sed 's|.*\/\(.*\)|\1|g')"
    dest_path="${DEST_DIR}/${file_path}"

    ##
    #echo "current file: ${current_file}"
    #echo "path        : ${file_path}"
    #echo "path        : ${file_name}"
    #echo "dest_path   : ${dest_path}"
    ##

    if [ ! -d ${dest_path} ]; then
        mkdir -p ${dest_path}
    fi

    cp -v ${SOURCE_DIR}/${current_file} ${dest_path}
    let "NB_FILES++"
done

NB_FILES_FOUND=$(find ${DEST_DIR} -type f | wc -l)

echo "==========================================================================="
echo "${NB_FILES} files copy under ${DEST_DIR}, found ${NB_FILES_FOUND} files under ${DEST_DIR}"
echo "==========================================================================="

