cd() {
    # Call the original cd command
    command cd "$@" 

    # Check if the current directory contains the .catkin_workspace file
    if [ -f ".catkin_workspace" ]; then

        CWD=$(pwd)/src
        if [[ ":$ROS_PACKAGE_PATH:" == *":$CWD:"* ]]; then
            # Current workspace has already been setup
            return
        fi

        echo "This Catkin workspace ($CWD) is NOT in ROS_PACKAGE_PATH!"
        echo "Do you want to source this workspace? (y/n)"
        read -r response

        # Check if the response is 'y' or 'Y'
        if [[ "$response" == "y" || "$response" == "Y" ]]; then
            source devel/setup.bash
        fi
    fi
}