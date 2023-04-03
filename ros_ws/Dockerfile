FROM nvcr.io/nvidia/l4t-tensorflow:r35.2.1-tf2.11-py3 as tensorflow
FROM dustynv/ros:humble-pytorch-l4t-r35.2.1

ENV ROS_DEPS_WS=/opt/ros_deps
ENV ROS_WS=/ros_ws
ENV PYTHON3_VERSION=3.8

# Get Tensorflow in here

#
# apt packages
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
		python3-dev \
		python3-matplotlib \
		build-essential \
		gfortran \
		git \
		cmake \
		curl \
		libopenblas-dev \
		liblapack-dev \
		libblas-dev \
		libhdf5-serial-dev \
		hdf5-tools \
		libhdf5-dev \
		zlib1g-dev \
		zip \
		libjpeg8-dev \
		libopenmpi3 \
		openmpi-bin \
		openmpi-common \
		protobuf-compiler \
		libprotoc-dev \
		llvm-9 \
		llvm-9-dev \
		libffi-dev \
		libsndfile1 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean


#
# pull protobuf-cpp from TF container
#
ENV PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp

COPY --from=tensorflow /usr/local/bin/protoc /usr/local/bin
COPY --from=tensorflow /usr/local/lib/libproto* /usr/local/lib/
COPY --from=tensorflow /usr/local/include/google /usr/local/include/google

#
# python packages from TF container
#

COPY --from=tensorflow /usr/local/lib/python2.7/dist-packages/ /usr/local/lib/python2.7/dist-packages/
COPY --from=tensorflow /usr/local/lib/python${PYTHON3_VERSION}/dist-packages/ /usr/local/lib/python${PYTHON3_VERSION}/dist-packages/

#
# python pip packages
#
RUN pip3 install --no-cache-dir --ignore-installed pybind11 
RUN pip3 install --no-cache-dir --verbose onnx
RUN pip3 install --no-cache-dir --verbose scipy
RUN pip3 install --no-cache-dir --verbose scikit-learn
RUN pip3 install --no-cache-dir --verbose pandas
RUN pip3 install --no-cache-dir --verbose pycuda
RUN pip3 install --no-cache-dir --verbose numba


#
# CuPy
#
ARG CUPY_VERSION=v11.5.0
ARG CUPY_NVCC_GENERATE_CODE="arch=compute_53,code=sm_53;arch=compute_62,code=sm_62;arch=compute_72,code=sm_72;arch=compute_87,code=sm_87"

RUN git clone -b ${CUPY_VERSION} --recursive https://github.com/cupy/cupy cupy && \
    cd cupy && \
    pip3 install --no-cache-dir fastrlock && \
    python3 setup.py install --verbose && \
    cd ../ && \
    rm -rf cupy


#
# PyCUDA
#
RUN pip3 uninstall -y pycuda
RUN pip3 install --no-cache-dir --verbose pycuda six


# ---- Now Build and Install Bearmax ROS Dependencies ----

# Copy requirements.txt
COPY ./src/bearmax/requirements.txt requirements.txt
# Install pip dependencies
RUN pip3 install jetson-stats && \
    pip3 install -r requirements.txt && \
    # Fix imutils bug
    cd "$(pip3 show imutils | grep Location | awk '{ print $2 }')/imutils/face_utils" && \
    sed -i 's@eyesCenter =.*@eyesCenter = (float((leftEyeCenter[0] + rightEyeCenter[0]) // 2),@;s@(leftEyeCenter\[1\] + rightEyeCenter\[1\]) // 2)@float((leftEyeCenter[1] + rightEyeCenter[1]) // 2))@' facealigner.py && \
    cd -

# Copy robot.rosinstall
COPY ./src/bearmax/robot.rosinstall robot.rosinstall

# Install dependencies
RUN \
    --mount=type=cache,target=/opt/ros_deps \
    . /opt/ros/humble/install/setup.sh && \
    mkdir -p $ROS_DEPS_WS/src && \
    vcs import $ROS_DEPS_WS/src < robot.rosinstall && \
    apt-get update && \
    apt-get install -y python3-pip && \
    rosdep update && \
    rosdep install -ry \
    --ignore-src \
    --from-paths $ROS_DEPS_WS/src \
    --rosdistro humble \
    --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv python-tensorflow-pip" && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \

    # Build Dependencies
    cd $ROS_DEPS_WS && \
    . /opt/ros/humble/install/setup.sh && \
    colcon build --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Remove build files (make the image smaller)
RUN \
    rm -rf $ROS_DEPS_WS/src && \
    rm -rf $ROS_DEPS_WS/logs && \
    rm -rf $ROS_DEPS_WS/build && \
    rm /robot.rosinstall

# Source ros deps from entrypoint
RUN sed -i '$iros_source_env "'${ROS_DEPS_WS}'/install/setup.bash"' /ros_entrypoint.sh

# ---- Build and install Bearmax -----

COPY ./src/ $ROS_WS/src/

WORKDIR $ROS_WS

RUN \
    --mount=type=cache,target=/ros_ws \
    . /opt/ros/humble/install/setup.sh && \
    . $ROS_DEPS_WS/install/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install -ry \
    --ignore-src \
    --from-paths $ROS_WS/src \
    --rosdistro humble \
    --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv python-tensorflow-pip" && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    # Build it
    colcon build \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ros deps from entrypoint
RUN sed -i '$iros_source_env "'${ROS_WS}'/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

STOPSIGNAL SIGINT

CMD ["ros2", "launch", "bearmax_bringup", "bearmax.launch.py"]
