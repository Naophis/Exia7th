#!/bin/bash

xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x4200ad31:0x3fca00500x4206bf69:0x3fca00b0