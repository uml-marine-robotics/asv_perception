# asv_perception
Perception for marine surface vehicles

## System Requirements
*  CPU:  i7 9th gen or later recommended
*  RAM:  8GB+
*  Video card:  5GB+ Video RAM, CUDA 10 compatible
*  Storage:  ~20GB for docker images and ML models
*  O/S:  Linux, Docker 18.03 or later, CUDA drivers
*  ROS Melodic host/image with a TF tree defined, including a fixed frame (eg, `odom`)
*  IMU publisher, ENU
*  RADAR image and pointcloud publisher
*  Camera image publisher, color

## Installation
*  `git clone --recursive https://github.com/uml-marine-robotics/asv_perception.git`
*  `cd asv_perception`
*  Put classifier data in `docker/classification/data`
*  Put segmentation data in `docker/segmentation/data`
*  `./build.sh`

## Setup/calibration
* TODO

## Running
`./run.sh` to start the `asv_perception` nodes, then run a `.bag` file or connect to asv sensors.  Use `rviz` and/or `rqt_image_view` to display output

## ROS Topics
* TODO

## File structure
*  `docker`:  Build files and data for docker images
*  `packages`:  ROS packages

## To-Do:
*  Obstacle tracking
    *  Camera: wasr/classifier offset
*  Homography:
    *  Camera-radar-pointcloud calibration (?)
    *  Heave via GPS data
*  Obstacle ID node
    *  Projection:  fix wasr/classifier offset, parent/child detections
    *  Accurate height/depth obstacle estimation
        * Height:  estimate from projection/geometry
        * Depth:   radar?