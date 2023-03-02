#!/bin/bash

xhost +local:

HOST_DIR=`dirname -- $(dirname -- $(readlink -f -- "$0"))`
WORK_DIR=/home/ros/ros_ws

docker run -it \
    --name rosgui \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v $HOST_DIR:$WORK_DIR \
    -v $HOME/.ros:/home/ros/.ros \
    -v $HOME/.rviz2:/home/ros/.rviz2 \
    -w $WORK_DIR \
    bearmax:ros \
    /bin/bash
