# Top-level CAMX driver's CMakeLists.txt

cmake_minimum_required(VERSION 2.6)

project (camx)

set (CAMX_OUT_PATH ${PROJECT_BINARY_DIR})
set (CAMX_CDK_PATH ${CMAKE_CURRENT_LIST_DIR}/../cdk)
set (CAMX_VENDOR_PATH ${CMAKE_CURRENT_LIST_DIR}/../vendor)
set (ANDROID_API 27)

if(${ANDROID_API} EQUAL 28)
add_definitions (-DCAMX_ANDROID_API=28)
else()
add_definitions (-DCAMX_ANDROID_API=27)
endif()

set (CAMX_LIB_OUTPUT_PATH camera/components)
# settings and SCOPE generation
# If adding outputs, add to custom_target autogen as well

include (${CMAKE_CURRENT_LIST_DIR}/common.cmake)

add_subdirectory (${CAMX_VENDOR_PATH}/chioverride/default/build/linuxembedded ${CAMX_OUT_PATH}/vendor)
add_subdirectory (${CAMX_VENDOR_PATH}/node/awbwrapper/build/linuxembedded ${CAMX_OUT_PATH}/awbwrapper)
add_subdirectory (${CAMX_VENDOR_PATH}/node/chiutils/build/linuxembedded ${CAMX_OUT_PATH}/vendor/chiutils)
add_subdirectory (${CAMX_VENDOR_PATH}/node/customhwnode/build/linuxembedded ${CAMX_OUT_PATH}/vendor/customhwnode)
add_subdirectory (${CAMX_VENDOR_PATH}/node/dummyrtb/build/linuxembedded ${CAMX_OUT_PATH}/vendor/dummyrtb)
add_subdirectory (${CAMX_VENDOR_PATH}/node/dummysat/build/linuxembedded ${CAMX_OUT_PATH}/vendor/dummysat)
add_subdirectory (${CAMX_VENDOR_PATH}/node/dummystich/build/linuxembedded ${CAMX_OUT_PATH}/vendor/dummystitch)
add_subdirectory (${CAMX_VENDOR_PATH}/node/hafoverride/build/linuxembedded ${CAMX_OUT_PATH}/vendor/hafoverride)
add_subdirectory (${CAMX_VENDOR_PATH}/node/hvx/addconstant/build/linuxembedded ${CAMX_OUT_PATH}/hvx/addconstant)
add_subdirectory (${CAMX_VENDOR_PATH}/node/hvx/binning/build/linuxembedded ${CAMX_OUT_PATH}/hvx/binning)
add_subdirectory (${CAMX_VENDOR_PATH}/node/gpu/build/linuxembedded ${CAMX_OUT_PATH}/vendor/gpu)
add_subdirectory (${CAMX_VENDOR_PATH}/node/memcpy/build/linuxembedded ${CAMX_OUT_PATH}/vendor/memcpy)
add_subdirectory (${CAMX_VENDOR_PATH}/node/staticaecalgo/build/linuxembedded ${CAMX_OUT_PATH}/staticaecalgo)
add_subdirectory (${CAMX_VENDOR_PATH}/node/staticafalgo/build/linuxembedded ${CAMX_OUT_PATH}/staticafalgo)
add_subdirectory (${CAMX_VENDOR_PATH}/node/staticawbalgo/build/linuxembedded ${CAMX_OUT_PATH}/staticawbalgo)
add_subdirectory (${CAMX_VENDOR_PATH}/node/staticpdlibalgo/build/linuxembedded ${CAMX_OUT_PATH}/staticpdlibalgo)
add_subdirectory (${CAMX_VENDOR_PATH}/node/remosaic/build/linuxembedded ${CAMX_OUT_PATH}/vendor/remosaic)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/imx318/build/linuxembedded ${CAMX_OUT_PATH}/sensor/imx318)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/imx334/build/linuxembedded ${CAMX_OUT_PATH}/sensor/imx334)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/imx362/build/linuxembedded ${CAMX_OUT_PATH}/sensor/imx362)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/imx386/build/linuxembedded ${CAMX_OUT_PATH}/sensor/imx386)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/imx577/build/linuxembedded ${CAMX_OUT_PATH}/sensor/imx577)

## ACHEUL
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/imx471/build/linuxembedded ${CAMX_OUT_PATH}/sensor/imx471)

add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/max7366_6dof/build/linuxembedded ${CAMX_OUT_PATH}/sensor/max7366_6dof)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/max7366_ov6211/build/linuxembedded ${CAMX_OUT_PATH}/sensor/max7366_ov6211)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/max7366_ov9282/build/linuxembedded ${CAMX_OUT_PATH}/sensor/max7366_ov9282)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/max7366_eyetrack/build/linuxembedded ${CAMX_OUT_PATH}/sensor/max7366_eyetrack)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/ov12a10/build/linuxembedded ${CAMX_OUT_PATH}/sensor/ov12a10)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/ov13855/build/linuxembedded ${CAMX_OUT_PATH}/sensor/ov13855)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/ov13880/build/linuxembedded ${CAMX_OUT_PATH}/sensor/ov13880)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/ov7251/build/linuxembedded ${CAMX_OUT_PATH}/sensor/ov7251)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/ov8856/build/linuxembedded ${CAMX_OUT_PATH}/sensor/ov8856)
add_subdirectory (${CAMX_VENDOR_PATH}/sensor/default/s5k2l7/build/linuxembedded ${CAMX_OUT_PATH}/sensor/s5k2l7)
