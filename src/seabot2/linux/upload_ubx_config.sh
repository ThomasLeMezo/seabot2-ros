#!/usr/bin/zsh
# Upload UBX config to GPS module

# GNSS device
GNSS_DEVICE="/dev/serial0"
FILENAME="/home/pi/config/default/ublox_m8n.txt"

# Check if the file is provided
FILENAME=$1
if [ -z $FILENAME ]; then
    echo "Usage: $0 <filename>"
    exit 1
fi

#Convert the file to binary
awk '{for (i=3; i<=NF; i++) printf "%s", $i} END {print ""}' $FILENAME | xxd -r -p > $FILENAME.bin

# Check if the file is converted
if [ ! -f $FILENAME.bin ]; then
    echo "Failed to convert the file to binary"
    exit 1
fi

# Check if the GPS module is connected
if [ ! -e $GNSS_DEVICE ]; then
    echo "GPS module is not connected"
    exit 1
fi

# Assuming the GPS module is connected to $GNSS_DEVICE
gpsctl -f $GNSS_DEVICE -x < $FILENAME.bin

# Clean up the binary file if exists
if [ -f $FILENAME.bin ]; then
    rm $FILENAME.bin
fi