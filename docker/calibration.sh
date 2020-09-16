#!/bin/sh
rqt_image_view &
python ${CATKIN_WS}/src/asv_perception_homography/src/asv_perception_homography/calibrate.py 