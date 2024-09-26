# Use the ROS Noetic base image
FROM ros:noetic

# Set the working directory inside the Docker container
WORKDIR /app

# Install any additional dependencies your ROS application might need
RUN apt update && apt install -y python3-pip ros-noetic-desktop-full
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN apt install -y byobu
RUN rosdep init && rosdep fix-permissions && rosdep update

# Setup command prompt colors
RUN echo "PS1='🐋 \e[1m\e[38;5;70m\u@\h\e[0m\e[1m:\e[38;5;67m\w\e[0m\e[1m#\e[0m '" >> /root/.bashrc

CMD ["byobu"]
