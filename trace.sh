#!/bin/bash

xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x40375bce:0x3fcf4a100x4037b1ed:0x3fcf4a30 0x4037e116:0x3fcf4a50 0x4037ceae:0x3fcf4ad0 0x4037b2a8:0x3fcf4af0 0x4037b29e:0x0000000d