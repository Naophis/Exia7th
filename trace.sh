#!/bin/bash

xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x42010726:0x3fca17700x42010a97:0x3fca1850 0x42010aa1:0x3fca1870 0x42010b5b:0x3fca18b0 0x42064bb9:0x3fca18f0