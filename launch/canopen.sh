#!/bin/sh
# Bind the USBCAN device /dev/ttyUSB1
sudo -S slcand -o -c -f -s6 tty_usb slcan0
echo "slcand ttyusb slcan0 "
sleep 0.5
sudo -S ifconfig slcan0 up
echo "ifconfig slcan0 up "
sleep 0.5
