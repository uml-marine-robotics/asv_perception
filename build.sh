#!/bin/bash

sudo docker build -t asv_perception_core -f Dockerfile-core --force-rm .
sudo docker build -t asv_perception_classification -f Dockerfile-classification --force-rm \
  --build-arg cuda=1 --build-arg cuda_tc=1 .
