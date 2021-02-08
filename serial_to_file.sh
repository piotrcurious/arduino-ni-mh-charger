#!/bin/bash
cat /dev/ttyUSB0 > data.raw
#minicom -D /dev/ttyUSB0 -b 56700 -C data.raw 
