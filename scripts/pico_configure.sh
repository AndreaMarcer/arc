#!/bin/bashmkdir ${ARC_DIR}/build

if [ ! -d "${ARC_DIR}/build" ]; then
  mkdir ${ARC_DIR}/build
fi

cd ${ARC_DIR}/build
cmake ..
cd -