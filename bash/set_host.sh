#!/bin/bash

# Base IP address for TurtleBot3
BASE_IP="192.168.28."
BASE_NUMBER=10  # Localhost is represented by 10, TurtleBots by 11-15

# Main script execution
echo "Select a TurtleBot3 number:"
echo "0 - localhost"
echo "1 - TurtleBot3 11"
echo "2 - TurtleBot3 12"
echo "3 - TurtleBot3 13"
echo "4 - TurtleBot3 14"
echo "5 - TurtleBot3 15"

read -p "Enter your choice [0-5]: " choice

# Calculate the last number for the IP and ROS master URI based on user choice
if [[ $choice -eq 0 ]]; then
    TURTLEBOT3_IP="localhost"  # Localhost
    TURTLEBOT3_NUMBER="$BASE_NUMBER"

    export ROS_HOSTNAME="localhost"
    export ROS_IP="localhost"
else
    TURTLEBOT3_IP="${BASE_IP}$(($BASE_NUMBER + $choice))"  # TurtleBot3 IPs from 11 to 15
    TURTLEBOT3_NUMBER="$(($BASE_NUMBER + $choice))"
    TURTLEBOT3_NAME="waffle$choice"

    IP_ADDRESS="172.19.228.77"
    export ROS_HOSTNAME="$IP_ADDRESS"
    export ROS_IP="$IP_ADDRESS"
fi

# Set the environment variables
export TURTLEBOT3_MODEL="waffle_pi"
export TURTLEBOT3_IP
export TURTLEBOT3_NUMBER
export ROS_MASTER_URI="http://${TURTLEBOT3_IP}:11311"

# IP_ADDRESS=$(hostname -I | awk '{print $1}')

echo "Environment variables set for TurtleBot3 with IP: $TURTLEBOT3_IP"
