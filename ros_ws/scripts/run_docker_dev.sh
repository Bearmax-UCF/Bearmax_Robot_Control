#!/bin/bash

xhost +local:

HOST_DIR=/tmp/placeholderdir
WORK_DIR=/mnt/

docker run -it --rm \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v $HOST_DIR:$WORK_DIR
    -w $WORK_DIR \
    ros:humble \
    /bin/bash
