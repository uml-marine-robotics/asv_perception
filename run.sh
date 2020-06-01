#!/bin/bash

echo "Starting asv_perception_core"
docker container stop core
docker run -d --rm \
  --name core \
  --network="host" \
  --env ROS_IP=0.0.0.0 \
  asv_perception_core

echo "Starting asv_perception_classification"
docker container stop classification
docker run -d --rm \
  --name classification \
  --gpus all \
  --network="host" --env ROS_MASTER_URI=http://0.0.0.0:11311/ \
  -v $(pwd)/docker/classification/data:/data \
  asv_perception_classification

echo "Starting asv_perception_segmentation"
docker container stop segmentation
docker run -d --rm \
  --name segmentation \
  --gpus all \
  --network="host" \
  --env ROS_MASTER_URI=http://0.0.0.0:11311/ \
  -v $(pwd)/docker/segmentation/data:/data \
  asv_perception_segmentation
