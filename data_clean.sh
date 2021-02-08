#!/bin/bash
mv -f device1.txt.old device1.txt.old.old
mv -f device1.txt device1.txt.old
cat data.raw | grep "DEVICE 1" > device1.txt
mv -f device2.txt.old device2.txt.old.old
mv -f device2.txt device2.txt.old
cat data.raw | grep "DEVICE 2" > device2.txt
