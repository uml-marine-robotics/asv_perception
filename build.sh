#!/bin/bash

# builds the asv_perception docker image
docker build -t asv_perception -f docker/Dockerfile --force-rm .