FROM tensorflow/tensorflow:1.15.0-gpu

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS
RUN set -euo pipefail && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get -y install --no-install-recommends python-opencv ros-melodic-perception python-rosinstall python-rosdep && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    pip install opencv-python scipy pillow && \
    rosdep init && \
    rosdep update

# set ROS distro
ARG ROS_DISTRO=melodic
ENV ROS_DISTRO=${ROS_DISTRO}

# Catkin workspace
ARG CATKIN_WS="/catkin_ws"

ENV CATKIN_WS=${CATKIN_WS}
WORKDIR ${CATKIN_WS}

# set entrypoint
ENTRYPOINT [ "/catkin_ws/src/ros_entrypoint.sh" ]

# set common python scripts
ENV PYTHONPATH "${PYTHONPATH}:${CATKIN_WS}/src/asv_perception_common/src"

# set startup script
CMD [ "sh", "-c", "roslaunch --wait src/asv_perception_segmentation/launch/master.launch" ]

# Install ROS pkgs
COPY common src
COPY segmentation/asv_perception_segmentation src/asv_perception_segmentation

# finalize install, make ros pkgs
RUN chmod +x ${CATKIN_WS}/src/ros_entrypoint.sh && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make
