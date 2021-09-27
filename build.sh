#!/bin/bash

# builds the asv_perception docker image
#docker build --no-cache -t asv_perception -f docker/Dockerfile --force-rm .
docker build -t asv_perception -f docker/Dockerfile --force-rm .