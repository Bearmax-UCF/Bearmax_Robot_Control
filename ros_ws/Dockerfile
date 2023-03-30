# FROM nvcr.io/nvidia/l4t-tensorflow:r35.2.1-tf2.11-py3 as tensorflow
FROM bearmax:ros as bearmax
FROM dustynv/ros:humble-pytorch-l4t-r35.2.1

# Install python dependencies
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip3 install -U jetson-stats imutils && \
    # Fix imutils bug
    cd "$(pip3 show imutils | grep Location | awk '{ print $2 }')/imutils/face_utils" && \
    sed -i 's@eyesCenter =.*@eyesCenter = (float((leftEyeCenter[0] + rightEyeCenter[0]) // 2),@;s@(leftEyeCenter\[1\] + rightEyeCenter\[1\]) // 2)@float((leftEyeCenter[1] + rightEyeCenter[1]) // 2))@' facealigner.py && \
    cd -

# Copy dependencies over
COPY --from=bearmax /opt/ros/humble/ /opt/ros/humble/install/

COPY --from=bearmax /ros_ws/ /ros_ws/

WORKDIR /ros_ws

# source ros from entrypoint
RUN sed --in-place --expression \
    '$isource "/ros_ws/install/setup.bash"' \
    /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

STOPSIGNAL SIGINT

# run main ros launch file
CMD ["ros2", "launch", "bearmax_bringup", "bearmax.launch.py"]
