# asv_perception
Perception for marine surface vehicles

## Installation
*  `git clone https://github.com/uml-marine-robotics/asv_perception.git`
*  Download `https://pjreddie.com/media/files/yolov3.weights` to `classification/data`
*  `cd asv_perception`
*  `./build.sh`

## Running
`./run.sh`, then run a `.bag` file or connect to asv sensors.  Use `rviz` to display output

## File structure
*  `common`:  Common ROS messages and python scripts
*  `core`:  Files/packages which do not need to be run in separate containers
*  `classification`:  Object classification/detection files/packages
*  `segmentation`:  Image segmentation files/packages

## TODO:
*  Video demo of pipeline
    *  Tom needs to finish WaSR node, ObstacleID node
*  Classification model (Rakshith)
    *  Currently implemented vanilla YOLOv3 ROS node/docker image, need domain-specific classifier
*  Image segmentation (WaSR)
    *  Convert to docker image + ROS node, a la classification node
*  Obstacle ID node
    *  Classification + segmentation fusion (convert Python node to C++ nodelet)
    *  Finish integration into pipeline
*  Incorporate IMU data into homography node
*  Remove `image_correction` node?
    *  WaSR needs input invariance
    *  Change homography params for new resolution
*  Remove need for separate classification container?  Need CUDA
    *  TBD - wait for Rakshith's solution
*  Improve performance of WaSR
