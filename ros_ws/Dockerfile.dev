FROM ros:humble

RUN groupadd -g 1000 ros && \
    useradd -m -u 1000 -g ros -G sudo ros && \
    echo "ros:ros" | chpasswd && \
    apt-get update && \
    echo ". /opt/ros/humble/setup.bash" >> /etc/profile

USER ros

RUN mkdir -p /home/ros/.ros && \
    mkdir -p /home/ros/.rviz2 && \
    mkdir -p /home/ros/ros_ws && \
    echo ". /opt/ros/humble/setup.bash" >> /home/ros/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/ros/.bashrc

# TODO: Install dependencies and ensure is ready and able to be run on robot

#RUN apt-get update && rosdep install -y

