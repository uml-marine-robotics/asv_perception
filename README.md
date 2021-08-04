# asv_perception
Perception for marine surface vehicles

## System Requirements
*  CPU:  i7 9th gen or later recommended
*  RAM:  8GB+
*  Video card:  5GB+ Video RAM, CUDA 10 compatible
*  Storage:  ~20GB for docker images and ML models
*  O/S:  Linux, Docker 18.03 or later, CUDA drivers
*  ROS Melodic host/image with a TF tree defined, including a fixed frame (eg, `odom`)
*  IMU publisher, ENU, type:  `sensor_msgs::Imu`
*  RADAR image, type `sensor_msgs::CompressedImage`
*  RADAR pointcloud publisher, type `sensor_msgs::PointCloud2`
*  Camera image publisher, color, type `sensor_msgs::CompressedImage`

## Conceptual overview
`asv_perception` contains ROS nodes specialized for image processing, pointcloud generation, obstacle creation, and obstacle tracking.  The expected inputs are described in the System Requirements and the fused/tracked obstacles are published in the `/tracking/fusion/obstacles` ROS topic.  See `packages/asv_perception_common/msg/Obstacle.msg` for a list of the fields which are included.

The system can currently accommodate visible light cameras and pointcloud-generating sensors (eg RADAR, LIDAR), and multiple sensors per type.  Additionally, an example tool is provided for publishing the obstacle information in JSON format to an external source/system over IP.  See the `Individual package description` section below additional details.

The default configuration is for a monocular camera, LIDAR, and two RADAR sensors, but can be adjusted to accommodate alternative sensor configurations.

## Installation
*  `git clone --recursive https://github.com/uml-marine-robotics/asv_perception.git`
*  `cd asv_perception`
*  Adjust launch files for your sensor configuration
*  Put classifier data in `data/classification/`
    *  To use Darknet MS-COCO pretrained weights:
        *  Download appropriate `cfg` and `weights` from https://github.com/AlexeyAB/darknet/wiki/YOLOv4-model-zoo to `model.cfg` and `model.weights`
        *  Download https://github.com/AlexeyAB/darknet/blob/master/cfg/coco.names to `model.names`
        *  Download https://github.com/AlexeyAB/darknet/blob/master/cfg/coco.data to `model.data`
            * In this file, change `names = ...` to `names = /data/classification/model.names`
*  (Optional) Put WaSR segmentation weights in `data/segmentation/` 
    *  Only required if using segmentation node with `USE_SEGMENTATION` environment variable
    *  Pretrained segmentation weights can be acquired from https://github.com/bborja/wasr_network
*  `./build.sh`

## Setup/calibration
* Calibration tool:
    * Start `asv_perception` image with the `VNC_PORT` environment variable specified (e.g. `VNC_PORT=5900`). In other words, execute run.sh by '. run.sh'. Make sure that the  
    docker container is running successfully.
    * Start robot sensor ROS drivers, or start a `.bag` file (rosbag play <bag file path>)
    * Connect to the asv_perception GUI with a vnc client, at `<docker_ip_address>::<VNC_PORT>`.  A desktop window should appear within VNC. If you are on Ubuntu Linux, 'vncviewer' should be available to you. If you are running the VNC client from the same machine as the docker container, then you need not give IP address. Just specifying port number is sufficient.
    * In the VNC desktop, click `Run Calibration`.  A `rqt_image_view` window and a `Calibration` window should appear
    * Change `rqt_image_view` to `cameraN/homography_vis/image`, where `cameraN` is the camera you wish to calibrate.  You should see the camera image along with the RADAR image overlay
    * In the `Calibration` window, set the topic to `cameraN/homography`, where `cameraN` is the camera you wish to calibrate.  
    * Adjust the radar image using the tuner until sufficient alignment is achieved.
    * When complete, save the calibration, then close all windows and exit VNC.

## Running
`./run.sh` to start the `asv_perception` nodes, then run a `.bag` file or connect to asv sensors.  Use `rviz` and/or `rqt_image_view` to display output.  See `./run.sh` for command line options.

## File structure
*  `docker`:  Build files and data for docker images
*  `packages`:  ROS packages
*  `data`:  Configuration data and deep learning models

## Individual package description
Each node may have multiple inputs, outputs, and parameters to control their behavior.  Below is a high level summary of each node's purpose and main ROS nodes.  See the referenced file headers for details on input/output topics and parameters.

* `asv_perception_common`:  Common messages and code shared by other packages
* `asv_perception_homography`:  Integrates IMU and camera-radar homography data.  Publishes homography matrix for camera to radar and supports the calibration tool
    * Node:  `nodes/homography.py`
    * Node:  `nodes/visualization.py`
    * Calibration tool:  See `Setup/calibration` section above
* `asv_perception_classification`:  Performs object detection in camera images and reports the ROI
    * Node:  `src/darknet_node.py`
* `asv_perception_segmentation`:  Semantic separation of camera image into water, sky, and obstacle components, publishing an image for each component.
    * Node:  `src/segmentation_node.py`
* `asv_perception_obstacleid`:  Classified obstacle projection, unclassified obstacle pixel projection and clustering, custom Point Cloud filtering and concatenation.
    * Node:  `include/ObstacleExtractionNodelet.h`, euclidean clustering and constructs `asv_perception_common/Obstacle` messages
    * Node:  `include/ObstacleProjectionNodelet.h`, classified bounding box adjustment, Obstacle message creation, unclassified pixel projection
    * Node:  `include/PointCloudConcatNodelet.h`, Concatenates multiple partial pointcloud segments received over time into a single pointcloud
    * Node:  `include/PointCloudFilterNodelet.h`, Custom pointcloud filtering, including min-distance, outlier removal, and cluster area inclusion/exclusion
* `asv_perception_tracking`:  Sensor-level obstacle tracking and fusion of multi-sensor obstacles.  RViz visualization of obstacles.  Reporting of obstacle data to external systems
    * Node:  `nodes/fusion.py`:  Fusion of tracked obstacle data from N sensors into singular output
    * Node:  `nodes/nmea_reporting.py`:  Example of obstacle reporting in NMEA format
    * Node:  `nodes/socket_reporting.py`:  Example of obstacle reporting in JSON format over UDP
    * Tool:  `nodes/socket_reporting_listener.py`:  Example of listener application which receives JSON data over UDP
    * Node:  `nodes/tracking.py`:  Node which performs sensor-level obstacle tracking
    * Node:  `nodes/visualization.py`:  Obstacle visualization in RViz
