#!/bin/bash

cd && arduino-cli compile  --output-dir  . -v  --fqbn  dadamachines_doppler:samd:Himalia  /PRJ/IOCore/Kalyke/
chmod 755 /root/uf2/utils/uf2conv.py
uf2/utils/uf2conv.py -b 0x4000 -c -o mybin.uf2 Kalyke.ino.bin
cp mybin.uf2 /USB/Himalia/
