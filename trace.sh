#!/bin/bash

xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x40375c5a:0x3fca1a700x4037acb1:0x3fca1a90 0x40380b66:0x3fca1ab0 0x4204a9ee:0x3fca1b20 0x420484bf:0x3fca1b40 0x4202ed0e:0x3fca1b60 0x420348d1:0x3fca1b80 0x420349e0:0x3fca1ba0 0x42034e7d:0x3fca1be0 0x4200f25c:0x3fca1c00 0x42010928:0x3fca1c90 0x42011523:0x3fca1d10 0x42011a39:0x3fca1d40 0x42067db1:0x3fca1d80