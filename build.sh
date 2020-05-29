#!/bin/bash

docker build -t asv_perception_core -f core/Dockerfile --force-rm .
docker build -t asv_perception_segmentation -f segmentation/Dockerfile --force-rm .

# classifier: gpu build
docker build -t asv_perception_classification -f classification/Dockerfile --force-rm .

# classifier: cpu build
# docker build -t asv_perception_classification -f classification/Dockerfile --force-rm \
#    --build-arg cuda=0 --build-arg cuda_tc=0 .
