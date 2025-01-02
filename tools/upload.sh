#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

USER=salico
IP=192.168.0.111 # change this as needed

BIN_PATH=$1
BIN_FILE="${BIN_PATH##*/}"

TARGET_DIR=/home/salico/build
COMMAND="st-flash --reset write firmware.bin 0x08000000"

if [[ "$#" -ne 1 ]]; then
    echo "USAGE: ./upload.sh <path_to_bin>"
    echo "   Typically, <path_to_bin> will be at .pio/build/nucleo_fxxxxxx/firmware.bin"
    exit 1
fi

scp $BIN_PATH $USER@$IP:$TARGET_DIR
ssh $USER@$IP "st-flash --reset write $TARGET_DIR/$BIN_FILE 0x08000000"
