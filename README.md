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
*  Obstacle reporting node
*  Classification node/model (Tom + Rakshith)
    *  Currently implemented vanilla YOLOv3 ROS node/docker image, need domain-specific classifier
    *  Use current Darknet + new weights
*  Obstacle ID node
    *  Projection:  fix wasr/classifier offset, parent/child detections
    *  Refactor pointcloud filtering to nodelet
    *  Accurate height/depth obstacle estimation
*  Incorporate IMU data into homography node
