// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#ifndef OBSTACLE_ID_DEFS_H
#define OBSTACLE_ID_DEFS_H

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

namespace obstacle_id {

    using point_type = pcl::PointXYZ;
    using pointcloud_type = pcl::PointCloud<point_type>;
    using image_type = cv::Mat;

}   // ns

#endif