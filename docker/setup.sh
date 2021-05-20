#!/bin/sh

# ROS sources.  lsb_release not included in cuda img
#echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list
echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# apt packages
apt-get update

apt-get -y install --no-install-recommends \
    ros-melodic-desktop-full \
    ros-melodic-perception \
    ros-melodic-robot-localization \
    ros-melodic-nmea-msgs \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-ros-numpy \
    python-pip \
    python-opencv \
    python-rosinstall \
    python-rosdep \
    python-tk \
    xvfb \
    dbus-x11 \
    x11-utils \
    x11vnc \
    net-tools \
    xfce4 \
    xfce4-terminal \
    nano

# apt cleanup
apt-get clean
rm -rf /var/lib/apt/lists/*

# python packages
# latest python 2.7 support:
#   pip: 20
#   opencv:  4.2.0.32
#   numpy: 1.16.6
#   tensorflow 1.15 for wasr
#   cupy 6
python -m pip install --upgrade \
    pip==20.* wheel

python -m pip install --use-feature=2020-resolver \
    numpy==1.16.6 \
    scikit-learn \
    scipy \
    opencv-python==4.2.0.32 \
    setuptools==41.* \
    tensorflow==1.15.* \
    filterpy \
    cupy-cuda100 \
    pyswarm

# ROS init
rosdep init
rosdep update

# darknet with cuda and cuda_tc
cd /tmp
git clone -n https://github.com/AlexeyAB/darknet.git && \
    cd darknet && \
    git checkout e08a818d5fd12f2f6e6e262267b132be195b40a5 && \
    sed -i -e "s!OPENMP=0!OPENMP=1!g" Makefile && \
    sed -i -e "s!AVX=0!AVX=1!g" Makefile && \
    sed -i -e "s!LIBSO=0!LIBSO=1!g" Makefile && \
    sed -i -e "s!GPU=0!GPU=1!g" Makefile && \
    sed -i -e "s!CUDNN=0!CUDNN=1!g" Makefile && \
    sed -i -e "s!CUDNN_HALF=0!CUDNN_HALF=1!g" Makefile && \
    make && \
    mv libdarknet.so /usr/lib && \
    rm -r /tmp/darknet
