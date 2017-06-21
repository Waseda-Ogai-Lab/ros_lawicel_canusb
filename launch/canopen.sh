#!/bin/sh
# Bind the USBCAN device
sudo -S slcand -o -c -f -s6 /dev/ttyUSB0 slcan0
echo "slcand ttyUSB0 slcan0 "
sleep 0.5
sudo -S ifconfig slcan0 up
echo "ifconfig slcan0 up "
sleep 0.5
