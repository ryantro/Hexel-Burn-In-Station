#!/bin/bash

# ------ NEW METHOD WITH UHUBCTL ------
# https://github.com/mvp/uhubctl#linux-usb-permissions

uhubctl -l 1-1 -a 2

# ------------- OLD METHOD -------------

# Power off USB devices
# echo '1-1' | sudo tee /sys/bus/usb/drivers/usb/unbind
# Wait 5 seconds
# sleep 5
# Power on USB devices
# echo '1-1' | sudo tee /sys/bus/usb/drivers/usb/bind
