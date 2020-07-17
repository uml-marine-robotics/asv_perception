FROM ros:melodic-perception-bionic

ENV DEBIAN_FRONTEND=noninteractive

# apt/pip setup
COPY docker/core/setup.sh /setup.sh
RUN chmod +x /setup.sh && . /setup.sh

# define workspace
ARG CATKIN_WS="/catkin_ws"
ENV CATKIN_WS=${CATKIN_WS}

WORKDIR ${CATKIN_WS}

# set entrypoint
ENTRYPOINT [ "/ros_entrypoint.sh" ]

# startup
CMD [ "sh", "-c", "roslaunch ${CATKIN_WS}/src/asv_perception_core.launch localization_enabled:=true set_sim_time:=true" ]

# copy ros pkgs & img files
COPY packages/asv_perception_common src/asv_perception_common
COPY packages/asv_perception_homography src/asv_perception_homography
COPY packages/asv_perception_obstacleid src/asv_perception_obstacleid
COPY packages/asv_perception_obstacle_tracking src/asv_perception_obstacle_tracking
COPY packages/asv_perception_core.launch src/asv_perception_core.launch
COPY packages/localization.launch src/localization.launch
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh

# finalize install, make ros pkgs
RUN chmod +x /ros_entrypoint.sh && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make