#!/bin/sh

# apt packages
apt-get update

apt-get -y install \
    ros-melodic-robot-localization \
    ros-melodic-nmea-msgs \
    ros-melodic-tf2-sensor-msgs \
    python-pip

apt-get clean
rm -rf /var/lib/apt/lists/*

# python packages
# ab3dmot:  don't use their requirements.txt for py 2.7
pip install \
    numpy \
    numba==0.43.1 \
    llvmlite==0.31.0 \
    filterpy==1.4.5 \
    scikit-learn==0.19.2 \
    matplotlib==2.2.3