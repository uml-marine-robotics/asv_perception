FROM ros:melodic-perception-bionic

# define workspace
ARG CATKIN_WS="/catkin_ws"
ENV CATKIN_WS=${CATKIN_WS}

WORKDIR ${CATKIN_WS}

# set entrypoint
ENTRYPOINT [ "/ros_entrypoint.sh" ]

# common python scripts
ENV PYTHONPATH "${PYTHONPATH}:${CATKIN_WS}/src/asv_perception_common/src"

# startup
CMD [ "sh", "-c", "roslaunch ${CATKIN_WS}/src/asv_perception_homography/launch/homography.launch" ]

# copy ros pkgs & docker files
COPY packages/asv_perception_common src/asv_perception_common
COPY packages/asv_perception_homography src/asv_perception_homography
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh

# finalize install, make ros pkgs
RUN chmod +x /ros_entrypoint.sh && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make