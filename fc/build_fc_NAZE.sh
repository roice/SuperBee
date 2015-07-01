#!/bin/bash

# Get the absolute path of this script
fctop=$(cd "$(dirname "$0")"; pwd)

# build cleanflight for NAZE
cd $fctop/cleanflight-1.9.0
make TARGET=NAZE
