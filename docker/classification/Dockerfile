FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS.  lsb_release not included in cuda img
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get -y install --no-install-recommends python-pip python-opencv ros-melodic-perception python-rosinstall python-rosdep nano ros-melodic-image-view && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    pip install opencv-python && \
    rosdep init && \
    rosdep update

#installing python3 and pip3
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.6 \
    wget \
    python3.6-dev \
    python3-pip \
    && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# for finding libs
ENV LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:/usr/lib/"

# Get and compile darknet, move libdarknet.so to library path

# Should CUDA be enabled?
ARG cuda=1
# Compile with support for Tensor Cores?
ARG cuda_tc=1

WORKDIR /src

#install marine-keras-yolo and install dependencies
#RUN cd /catkin_ws/src && \
#    git clone https://github.com/rakshithsinghgm/Marine-Keras-Yolo.git && \
#    cd Marine-Keras-Yolo && \
#    pip3 install --upgrade setuptools && \
#    pip3 install -r requirements.txt

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

RUN cd /catkin_ws/src && \
    git clone https://github.com/rakshithsinghgm/Marine-Keras-Yolo.git && \
    cd Marine-Keras-Yolo && \
    wget http://rakshithsingh.com/model-download/ && \
    mv index.html keras_model.h5 && \
    pip3 install --upgrade setuptools && \
    pip3 install -r requirements.txt
