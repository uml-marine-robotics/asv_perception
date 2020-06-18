# asv_perception
Perception for marine surface vehicles

## Installation
*  `git clone https://github.com/uml-marine-robotics/asv_perception.git`
*  `cd asv_perception`
*  Put classifier data in `docker/classification/data`
*  Put segmentation data in `docker/segmentation/data`
*  `./build.sh`

## Running
`./run.sh` to start the `asv_perception` nodes, then run a `.bag` file or connect to asv sensors.  Use `rviz` and/or `rqt_image_view` to display output

## ROS Topics
* TODO

## File structure
*  `docker`:  Build files and data for docker images
*  `packages`:  ROS packages

## TODO:
*  Homography:
    *  Incorporate IMU data
    *  Online calibration
*  Basic obstacle tracking across frames; compute velocity vector
*  Obstacle reporting node
*  Obstacle ID node
    *  Projection:  fix wasr/classifier offset, parent/child detections
    *  Refactor pointcloud filtering to nodelet
    *  Accurate height/depth obstacle estimation
        * Height:  estimate from projection/geometry
        * Depth:   radar?