#!/bin/bash

# Check if TURTLEBOT3_IP is set
if [ -z "$TURTLEBOT3_IP" ]; then
  echo "Error: TURTLEBOT3_IP environment variable is not set."
  exit 1
fi

# SSH into the robot using the IP from TURTLEBOT3_IP
ssh user@$TURTLEBOT3_IP