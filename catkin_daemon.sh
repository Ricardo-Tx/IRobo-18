cd() {
    # Call the original cd command
    command cd "$@" 

    # Check if the current directory contains the .catkin_workspace file
    if [ -f ".catkin_workspace" ]; then

        CWD=$(pwd)/src
        DEVEL=$(pwd)/devel 

        if [[ ":$ROS_PACKAGE_PATH:" != *":$CWD:"* ]]; then
            # Current workspace has not been setup
            echo "This Catkin workspace ($CWD) is NOT in ROS_PACKAGE_PATH!"
            echo "Do you want to source this workspace? (y/n)"
            read -r response

            # Check if the response is 'y' or 'Y'
            if [[ "$response" == "y" || "$response" == "Y" ]]; then
                source devel/setup.bash
            fi
        fi


        for PKG_DIR in "$CWD"/*; do
            if [ -d "$PKG_DIR" ]; then
                # Get the package name (which is the directory name)
                PKG_NAME=$(basename "$PKG_DIR")

                # Check if there are any binaries or libraries in the devel/lib folder
                LIB_DIR="$DEVEL/lib/$PKG_NAME"
                INCLUDE_DIR="$DEVEL/include/$PKG_NAME"
                SHARE_DIR="$DEVEL/share/$PKG_NAME"
                
                # Check if binaries or other install files exist
                # if [ -d "$BIN_DIR" ] || [ -d "$INCLUDE_DIR" ] || [ -d "$SHARE_DIR" ]; then
                #     echo "Package $PKG_NAME has binaries or install files."
                # else
                #     echo "Package $PKG_NAME has NO binaries installed."
                # fi
                if [ ! -d "$LIB_DIR" ] || [ -z "$(ls -A "$LIB_DIR")" ]; then
                    echo "!!! Package $PKG_NAME has NO binaries installed."
                fi
            fi
        done
        
    fi
}