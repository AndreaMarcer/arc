#!/bin/bash

if [ -d "${ARC_DIR}/build" ]; then
    cd ${ARC_DIR}/build
    make -j4
    cd -
else
    echo "${ARC_DIR}/build does not exist. Please run pico_configure"
fi
