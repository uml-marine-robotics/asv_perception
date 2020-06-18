FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS.  lsb_release not included in cuda img
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get -y install --no-install-recommends python-pip python-opencv ros-melodic-perception python-rosinstall python-rosdep && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    pip install opencv-python && \
    rosdep init && \
    rosdep update

# for finding libs
ENV LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:/usr/lib/"

# Get and compile darknet, move libdarknet.so to library path

# Should CUDA be enabled?
ARG cuda=1
# Compile with support for Tensor Cores?
ARG cuda_tc=1

WORKDIR /src
RUN git clone -n https://github.com/AlexeyAB/darknet.git && \
    cd darknet && \
    git checkout e08a818d5fd12f2f6e6e262267b132be195b40a5 && \
    sed -i -e "s!OPENMP=0!OPENMP=1!g" Makefile && \
    sed -i -e "s!AVX=0!AVX=1!g" Makefile && \
    sed -i -e "s!LIBSO=0!LIBSO=1!g" Makefile && \
    sed -i -e "s!GPU=0!GPU=${cuda}!g" Makefile && \
    sed -i -e "s!CUDNN=0!CUDNN=${cuda}!g" Makefile && \
    sed -i -e "s!CUDNN_HALF=0!CUDNN_HALF=${cuda_tc}!g" Makefile && \
    make && \
    mv libdarknet.so /usr/lib && \
    rm -r /src

# set ROS distro
ARG ROS_DISTRO=melodic
ENV ROS_DISTRO=${ROS_DISTRO}

# Catkin workspace
ARG CATKIN_WS="/catkin_ws"

ENV CATKIN_WS=${CATKIN_WS}
WORKDIR ${CATKIN_WS}

# set entrypoint
ENTRYPOINT [ "/ros_entrypoint.sh" ]

# set common python scripts
ENV PYTHONPATH "${PYTHONPATH}:${CATKIN_WS}/src/asv_perception_common/src"

# set startup script
CMD [ "sh", "-c", "roslaunch --wait ${CATKIN_WS}/src/asv_perception_classification/launch/classification.launch" ]

# Install ROS pkgs & docker files
COPY packages/asv_perception_common src/asv_perception_common
COPY packages/asv_perception_classification src/asv_perception_classification
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh

# finalize install, make ros pkgs
RUN chmod +x /ros_entrypoint.sh && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make
