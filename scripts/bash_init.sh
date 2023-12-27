#! /bin/bash

export WORKING_DIRECTORY=${HOME}/workspace
export PICO_SDK_PATH=${WORKING_DIRECTORY}/arc/external/pico-sdk

alias arc='cd ${WORKING_DIRECTORY}/arc'
alias tmux='tmux -f ${WORKING_DIRECTORY}/arc/config/tmux.conf'
