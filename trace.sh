#!/bin/bash

xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x4200e75b:0x3fca1c200x42011537:0x3fca1ca0 0x42011555:0x3fca1cc0 0x42011f47:0x3fca1d00 0x420683ad:0x3fca1d40 