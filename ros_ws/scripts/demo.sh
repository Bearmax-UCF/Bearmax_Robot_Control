#!/bin/bash

trap 'trap - SIGINT SIGHUP EXIT && [ "$(jobs -p)" ] && kill -INT $(jobs -p)' SIGINT SIGHUP EXIT

while true; do
    # Reset
    ros2 action send_goal task bearmax_msgs/action/Task '{task_name: "reset"}'

    # Quizzical
    ros2 action send_goal task bearmax_msgs/action/Task '{task_name: "quizzical"}'
    sleep 2
    ros2 action send_goal task bearmax_msgs/action/Task '{task_name: "quizzical"}' 

    # Happy
    ros2 action send_goal task bearmax_msgs/action/Task '{task_name: "happy"}'

    # Sad
    ros2 action send_goal task bearmax_msgs/action/Task '{task_name: "sad"}' &
    sleep 2
    kill -INT %1

    # Angry
    ros2 action send_goal task bearmax_msgs/action/Task '{task_name: "angry"}' &
    sleep 2
    kill -INT %1

done
