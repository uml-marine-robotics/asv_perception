#!/bin/bash

docker run -d --rm --network="host" \
  --env ROS_IP=0.0.0.0 asv_perception_core

docker run -d --rm --gpus all --network="host" \
  --env ROS_MASTER_URI=http://0.0.0.0:11311/ asv_perception_classification
