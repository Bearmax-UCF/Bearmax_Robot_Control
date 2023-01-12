FROM ros:humble

RUN groupadd -g 1000 ros && \
    useradd -m -u 1000 -g ros -G sudo ros && \
    echo "ros:ros" | chpasswd && \
    apt-get update

USER ros

RUN mkdir -p /home/ros/.ros && \
    mkdir -p /home/ros/.rviz2 && \
    mkdir -p /home/ros/ros_ws

# TODO: Install dependencies and ensure is ready and able to be run on robot

#RUN apt-get update && rosdep install -y

