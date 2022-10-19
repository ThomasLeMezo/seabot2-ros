#!/usr/bin/zsh

# https://raspberrypi-guide.github.io/electronics/power-consumption-tricks

## Turn off USB/LAN
echo '1-1' |sudo tee /sys/bus/usb/drivers/usb/unbind
# echo '1-1' |sudo tee /sys/bus/usb/drivers/usb/bind

## Turn of HDMI output
sudo /opt/vc/bin/tvservice -o
#sudo /opt/vc/bin/tvservice -p

