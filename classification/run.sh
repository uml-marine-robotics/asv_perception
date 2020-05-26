#!/bin/bash
cd ${CATKIN_WS}
source devel/setup.bash
roslaunch --wait src/asv_perception_classification/launch/master.launch

