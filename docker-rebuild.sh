#!/bin/bash

# Build the Docker image
sudo docker build -t ros-noetic-env .

# Remove the existing Docker container if it exists
sudo docker container rm -f ros-noetic-container

clear

# Run the Docker container with the specified volume mount
sudo docker run -it --name ros-noetic-container -v $(pwd):/app ros-noetic-env