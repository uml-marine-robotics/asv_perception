# asv_perception
Perception for marine surface vehicles

## Installation
*  `git clone https://github.com/uml-marine-robotics/asv_perception.git`
*  Download classifier data to `docker/classification/data`
*  Download segmentation data to `docker/segmentation/data`
*  `cd asv_perception`
*  `./build.sh`

## Running
`./run.sh` to start the `asv_perception` nodes, then run a `.bag` file or connect to asv sensors.  Use `rviz` to display output

## ROS Topics
* TODO

## File structure
*  `docker`:  Build files and data for docker images
*  `packages`:  ROS packages

## TODO:
*  Video demo of pipeline
    *  Tom needs to finish ObstacleID node
*  Classification node/model (Rakshith)
    *  Currently implemented vanilla YOLOv3 ROS node/docker image, need domain-specific classifier
*  Obstacle ID node
    *  Classification + segmentation fusion (convert Python node to C++ nodelet)
    *  Finish integration into pipeline
*  Incorporate IMU data into homography node
*  Remove need for separate classification container?  Need CUDA
    *  TBD - wait for Rakshith's solution
