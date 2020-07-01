FROM ros:melodic-perception-bionic

ENV DEBIAN_FRONTEND=noninteractive

# Install packages
RUN apt-get update && \
    apt-get -y install ros-melodic-robot-localization ros-melodic-nmea-msgs && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# define workspace
ARG CATKIN_WS="/catkin_ws"
ENV CATKIN_WS=${CATKIN_WS}

WORKDIR ${CATKIN_WS}

# set entrypoint
ENTRYPOINT [ "/ros_entrypoint.sh" ]

# common python scripts
ENV PYTHONPATH "${PYTHONPATH}:${CATKIN_WS}/src/asv_perception_common/src"

# startup
CMD [ "sh", "-c", "roslaunch ${CATKIN_WS}/src/asv_perception_core.launch localization_enabled:=true set_sim_time:=true" ]

# copy ros pkgs & docker img files
COPY packages/asv_perception_common src/asv_perception_common
COPY packages/asv_perception_homography src/asv_perception_homography
COPY packages/asv_perception_obstacleid src/asv_perception_obstacleid
COPY packages/asv_perception_core.launch src/asv_perception_core.launch
COPY packages/localization.launch src/localization.launch
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh

# finalize install, make ros pkgs
RUN chmod +x /ros_entrypoint.sh && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make