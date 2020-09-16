FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

# apt & pip setup
COPY docker/setup.sh /setup.sh
RUN chmod +x /setup.sh && . /setup.sh

# set ROS distro
ARG ROS_DISTRO=melodic
ENV ROS_DISTRO=${ROS_DISTRO}

# for finding darknet libs
ENV LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:/usr/lib/"

# env var for finding config data
ENV ASV_PERCEPTION_CONFIG "/data/config/"

# define workspace
ARG CATKIN_WS="/catkin_ws"
ENV CATKIN_WS=${CATKIN_WS}

WORKDIR ${CATKIN_WS}

# copy ros pkgs & img files
COPY packages/ src/
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
COPY docker/startup.sh /startup.sh
COPY docker/.xinitrc /root/.xinitrc
COPY docker/calibration.sh /calibration.sh
COPY docker/calibration.desktop /root/Desktop/calibration.desktop

# finalize install, make ros pkgs
RUN chmod +x /ros_entrypoint.sh /startup.sh /calibration.sh /root/.xinitrc /root/Desktop/calibration.desktop && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make

# set entrypoint, startup
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "sh", "-c", "/startup.sh" ]