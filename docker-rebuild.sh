#!/bin/bash

# Remove the existing Docker container if it exists
sudo docker container rm -f ros-noetic-container

# Build the Docker image
sudo docker build -t ros-noetic-env .

# Run the Docker container with the specified volume mount
sudo docker run -it --env-file .env --name ros-noetic-container -v $(pwd):/app -v /tmp/.X11-unix:/tmp/.X11-unix --network=host ros-noetic-env