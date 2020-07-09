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
ENTRYPOINT [ "/ros_entrypoint.sh" ]

# set startup script
CMD [ "sh", "-c", "roslaunch --wait ${CATKIN_WS}/src/asv_perception_segmentation/launch/segmentation.launch" ]

# Install ROS pkgs & docker files
COPY packages/asv_perception_common src/asv_perception_common
COPY packages/asv_perception_segmentation src/asv_perception_segmentation
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh

# finalize install, make ros pkgs
RUN chmod +x /ros_entrypoint.sh && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make
