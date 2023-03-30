# FROM nvcr.io/nvidia/l4t-tensorflow:r35.2.1-tf2.11-py3 as tensorflow
FROM dustynv/ros:humble-pytorch-l4t-r35.2.1

# 
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Bring over our code
RUN mkdir -p /ros_ws
WORKDIR /ros_ws

# Install python dependencies
RUN apt-get update && \
    rosdep update && \
    apt-get install -y python3-pip && \
    pip3 install -U jetson-stats && \
    # Fix imutils bug
    cd "$(pip show imutils | grep Location | awk '{ print $2 }')/imutils/face_utils" && \
    sed -i 's@eyesCenter =.*@eyesCenter = (float((leftEyeCenter[0] + rightEyeCenter[0]) // 2),@;s@(leftEyeCenter\[1\] + rightEyeCenter\[1\]) // 2)@float((leftEyeCenter[1] + rightEyeCenter[1]) // 2))@' facealigner.py && \
    cd -

# Install bearmax dependencies
COPY ./scripts/install_rosdeps.sh ./install_rosdeps.sh
RUN \
    --mount=type=cache,target=/var/cache/apt \
    apt-get update && ./install_rosdeps.sh

COPY . .

# Build Bearmax ROS
RUN \
    --mount=type=cache,target=/ros_ws/build \
    --mount=type=cache,target=/ros_ws/install \
    . /opt/ros/humble/install/setup.sh && \
    colcon build --symlink-install --continue-on-error \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# source ros from entrypoint
RUN sed --in-place --expression \
    '$isource "/ros_ws/install/setup.bash"' \
    /ros_entrypoint.sh


ENTRYPOINT ["/ros_entrypoint.sh"]

STOPSIGNAL SIGINT

# run main ros launch file
CMD ["ros2", "launch", "bearmax_bringup", "bearmax.launch.py"]
