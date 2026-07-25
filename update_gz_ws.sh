#!/bin/bash
TARGET="/home/ace/gz_ws/src/ardupilot_gazebo/models/qr_target/model.sdf"
if [ -w "$TARGET" ]; then
    sed -i 's/<size>1 2 0.001<\/size>/<size>1 1 0.001<\/size>/g' "$TARGET"
    echo "Successfully updated QR code to 1m x 1m in $TARGET"
else
    echo "Cannot write to $TARGET from here. You must run this command in your main terminal:"
    echo "sed -i 's/<size>1 2 0.001<\/size>/<size>1 1 0.001<\/size>/g' \"$TARGET\""
fi
