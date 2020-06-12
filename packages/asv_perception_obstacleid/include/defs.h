#ifndef OBSTACLE_ID_DEFS_H
#define OBSTACLE_ID_DEFS_H

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

namespace obstacle_id {

    using point_type = pcl::PointXYZL;
    using pointcloud_type = pcl::PointCloud<point_type>;
    using image_type = cv::Mat;

}   // ns

#endif