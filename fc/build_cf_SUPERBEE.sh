#!/bin/bash

# Get the absolute path of this script
fctop=$(cd "$(dirname "$0")"; pwd)

# Build cleanflight for NAZE board
cd $fctop/cleanflight
#make clean
make TARGET=SUPERBEE
