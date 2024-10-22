#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <directory_path>"
    exit 1
fi

# If you can't edit files in vscode run this bash file
sudo chown -R $USER $1