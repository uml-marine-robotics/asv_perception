#ifndef OBSTACLEID_UTILS_H
#define OBSTACLEID_UTILS_H

#include <sensor_msgs/RegionOfInterest.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/PointCloud2.h>

#include "defs.h"

namespace obstacle_id {
namespace utils {

// convert ros roi to opencv rect, clamped to image shape
inline cv::Rect to_cv_rect( const sensor_msgs::RegionOfInterest& roi, const image_type& img ) {
    cv::Rect result = {}; // cv::Rect( (int)roi.x_offset, (int)roi.y_offset, (int)roi.width, (int)roi.height );
    result.x = std::min( (int)roi.x_offset, img.cols - 1);
    result.y = std::min( (int)roi.y_offset, img.rows - 1);
    result.width = std::min( (int)roi.width, img.cols - result.x );
    result.height = std::min( (int)roi.height, img.rows - result.y );
    return result;
}

// based on https://stackoverflow.com/q/35669182/882436
template <typename PointT, typename Alloc>
inline std::pair<PointT, PointT> minmax_3d( const std::vector<PointT, Alloc>& points ) {

    if ( points.empty() )
        return {};

    auto
        min = points.front()
        , max = points.front()
        ;

    for ( const auto& pt : points ) {
        min.x = std::min( min.x, pt.x );
        min.y = std::min( min.y, pt.y );
        min.z = std::min( min.z, pt.z );
        max.x = std::max( max.x, pt.x );
        max.y = std::max( max.y, pt.y );
        max.z = std::max( max.z, pt.z );
    }

    return std::make_pair(std::move(min), std::move(max));
}

// test whether a PointCloud2 is valid, based on 
//  https://github.com/ros-perception/perception_pcl/blob/melodic-devel/pcl_ros/include/pcl_ros/pcl_nodelet.h
inline bool is_cloud_valid( const sensor_msgs::PointCloud2::ConstPtr& cloud ) {
    return 
        ( cloud )
        && ( cloud->width * cloud->height * cloud->point_step == cloud->data.size() )
        ;
}

// convert point type to ros point32
inline geometry_msgs::Point32 to_point32( const point_type& pt ) {
    geometry_msgs::Point32 result = {};
    result.x = pt.x;
    result.y = pt.y;
    result.z = pt.z;
    return result;
}

}}  // ns
#endif