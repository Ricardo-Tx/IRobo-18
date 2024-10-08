#!/bin/bash

# Base IP address for TurtleBot3
BASE_IP="192.168.28."

# Main script execution
echo "Select a TurtleBot3 number to ping:"
echo "1 - TurtleBot3 11"
echo "2 - TurtleBot3 12"
echo "3 - TurtleBot3 13"
echo "4 - TurtleBot3 14"
echo "5 - TurtleBot3 15"

read -p "Enter your choice [0-5]: " choice

IP_ADDRESS="${BASE_IP}$(($choice+10)):11311"

ping -c 1 $IP_ADDRESS &> /dev/null  # Ping the device with one packet
if [ $? -eq 0 ]; then
    echo "Device with IP $IP_ADDRESS is reachable."
else
    echo "Device with IP $IP_ADDRESS is not reachable."
fi
