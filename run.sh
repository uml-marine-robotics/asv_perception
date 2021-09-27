#!/bin/bash

# start asv_perception image
# optional environment variables:
#   VNC_PORT:  default=none.  If provided, enables connection to desktop within asv_perception via VNC on specified port
#   USE_LOCALIZATION:  0/1, default=0 (false).  Use localization package within this image.  If false, must provide TF elsewhere
#   USE_SIM_TIME: 0/1, default=0 (false).  Use when playing back .bag files to fix transformation time errors
#       When using sim time with rosbag play, you must include the --clock option in the rosbag play command
#   USE_SEGMENTATION: 0/1, default=0 (false).  Enable image segmentation nodes
# to avoid writing as a root
echo "Starting asv_perception"
docker container stop asv_perception
docker run -it --rm \
  --name asv_perception \
  --gpus all \
  --network="host" \
  --env ROS_IP=0.0.0.0 \
  --env VNC_PORT=5950 \
  --env USE_SEGMENTATION=1 \
  --env USE_LOCALIZATION=1 \
  --env USE_SIM_TIME=1 \
  -v $(pwd)/data/:/data \
  asv_perception