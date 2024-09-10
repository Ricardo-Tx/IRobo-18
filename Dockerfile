# Use the ROS Noetic base image
FROM ros:noetic

# Set the working directory inside the Docker container
WORKDIR /app

# Install any additional dependencies your ROS application might need
RUN apt update && apt install -y python3-pip ros-noetic-desktop-full

ENV TERM=xterm-256color
RUN echo "PS1='🐋 \e[1m\e[38;5;70m\u@\h\e[0m\e[1m:\e[1;38;5;67m\w\e[0m\e[1m#\e[0m '" >> /root/.bashrc