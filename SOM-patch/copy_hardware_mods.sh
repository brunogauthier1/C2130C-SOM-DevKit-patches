#!/bin/bash

SOURCE_DIR="/media/bruno/T7/devkit-1.1-FC/LINUX/android/hardware"

SOURCE_FILES+="qcom/display/composer/hwc_debugger.cpp"

DEST_DIR="./hardware"
NB_FILES=0

for current_file in ${SOURCE_FILES} ;
do
    file_path="$(echo ${current_file} | sed 's|\(.*\)\/.*|\1|g')"
    file_name="$(echo ${current_file} | sed 's|.*\/\(.*\)|\1|g')"
    dest_path="${DEST_DIR}/${file_path}/hardware"

    ##
    #echo "current file: ${current_file}"
    #echo "path        : ${file_path}"
    #echo "path        : ${file_name}"
    #echo "dest_path   : ${dest_path}"
    ##

    if [ ! -d ${dest_path} ]; then
        mkdir -p ${dest_path}
    fi

    if [ -f ${dest_path}/${current_file} ]; then
        echo "the file ${dest_path}/${current_file} already exist, won't copy"
    fi

    cp -v ${SOURCE_DIR}/${current_file} ${dest_path}
    let "NB_FILES++"
done

NB_FILES_FOUND=$(find ${DEST_DIR} -type f | wc -l)

echo "==========================================================================="
echo "${NB_FILES} files copy under ${DEST_DIR}, found ${NB_FILES_FOUND} files under ${DEST_DIR}"
echo "==========================================================================="

