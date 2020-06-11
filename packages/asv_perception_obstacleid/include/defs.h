#ifndef OBSTACLE_ID_DEFS_H
#define OBSTACLE_ID_DEFS_H

#include <map>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <asv_perception_common/ClassificationArray.h>
#include <asv_perception_common/Homography.h>

namespace obstacle_id {

    using point_type = pcl::PointXYZL;
    using pointcloud_type = pcl::PointCloud<point_type>;
    using image_type = cv::Mat;
    using classification2d_type = asv_perception_common::Classification;
    using classification2d_vector_type = asv_perception_common::ClassificationArray;

}   // ns

#endif