#!/bin/bash

binary=`find ./build -maxdepth 1 | grep bin$ `
partition=`find ./build/partition_table -maxdepth 1 | grep bin$ `
bootloader=`find ./build/bootloader -maxdepth 1 | grep bin$ `
echo ${binary}
echo ${partition}
echo ${bootloader}
esptool.py --chip esp32s3 -p /dev/ttyUSB1 -b 2000000 --before=default_reset --after=hard_reset write_flash --flash_mode dio \
--flash_freq 80m --flash_size 8MB \
0x0 ${bootloader} \
0x8000 ${partition} \
0x10000 ${binary}