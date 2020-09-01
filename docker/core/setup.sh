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
pip install \
    numpy \
    filterpy \
    scikit-learn