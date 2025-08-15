#!/bin/bash

git clone --recursive https://github.com/ETH-PBL/Nano_Swarm_Mapping.git
cd ~/SAFMC/Nano_Swarm_Mapping/crazyflie-firmware
git apply ./../firmware.patch
cd ~/SAFMC/Nano_Swarm_Mapping/swarm-firmware
sed -i 's/^CFLAGS += -Werror$/CFLAGS += -Wno-error/' ~/SAFMC/Nano_Swarm_Mapping/crazyflie-firmware/Makefile
sed -i 's/\(#define[ \t]*FREERTOS_HEAP_SIZE[ \t]*\)30000/\1 20000/' ~/SAFMC/Nano_Swarm_Mapping/crazyflie-firmware/src/config/config.h
make clean all -j8
