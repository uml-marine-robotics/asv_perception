#!/bin/bash

docker build -t asv_perception_core -f docker/core/Dockerfile --force-rm .
docker build -t asv_perception_segmentation -f docker/segmentation/Dockerfile --force-rm .

# classifier: gpu build
docker build -t asv_perception_classification -f docker/classification/Dockerfile --force-rm .

# classifier: cpu build
# docker build -t asv_perception_classification -f docker/classification/Dockerfile --force-rm \
#    --build-arg cuda=0 --build-arg cuda_tc=0 .
