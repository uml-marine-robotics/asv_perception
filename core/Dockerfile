FROM ros:melodic-perception-bionic

# define workspace
ARG CATKIN_WS="/catkin_ws"
ENV CATKIN_WS=${CATKIN_WS}

WORKDIR ${CATKIN_WS}

# set entrypoint
ENTRYPOINT [ "/catkin_ws/src/ros_entrypoint.sh" ]

# common python scripts
ENV PYTHONPATH "${PYTHONPATH}:${CATKIN_WS}/src/asv_perception_common/src"

# startup, include roscore if needed
CMD [ "sh", "-c", "roslaunch src/asv_perception_core/launch/master.launch" ]

# copy ros pkgs
COPY common src
COPY core src

# catkin_make packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && catkin_make