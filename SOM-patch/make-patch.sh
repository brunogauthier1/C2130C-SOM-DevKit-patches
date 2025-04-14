#!/bin/bash
ORIGIN_DIR=/media/bruno/T7-DevKit/devkit-1.1-FC.virgin/LINUX/android

diff -ru --unidirectional-new-file ${ORIGIN_DIR}/kernel   kernel/   > kernel.patch
diff -ru --unidirectional-new-file ${ORIGIN_DIR}/vendor   vendor/   > vendor.patch
diff -ru --unidirectional-new-file ${ORIGIN_DIR}/hardware hardware/ > hardware.patch

