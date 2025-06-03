#!/bin/bash

# Script to configure a specific CAN interface passed as an argument
IFACE=$1
BITRATE=1000000 # Adjust the bitrate as needed

/sbin/ip link set $IFACE up type can bitrate $BITRATE
/sbin/ip link set $IFACE txqueuelen 1000