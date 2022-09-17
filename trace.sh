#!/bin/bash

xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x42007c74:0x3fc9dae00x4200adc2:0x3fc9db00 0x42066649:0x3fc9db40