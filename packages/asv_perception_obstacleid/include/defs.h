#ifndef OBSTACLE_ID_DEFS_H
#define OBSTACLE_ID_DEFS_H

#include <pcl/pcl_base.h>

namespace obstacle_id {

    using point_type = pcl::PointXYZL;
    using pointcloud_type = pcl::PointCloud<point_type>;

}   // ns

#endif