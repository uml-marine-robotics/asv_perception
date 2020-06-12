#ifndef OBSTACLEID_UTILS_H
#define OBSTACLEID_UTILS_H

#include <sensor_msgs/RegionOfInterest.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include "defs.h"

namespace obstacle_id {
namespace utils {

// convert ros roi to opencv rect
inline cv::Rect to_cv_rect( const sensor_msgs::RegionOfInterest& roi ) {
    return cv::Rect( (int)roi.x_offset, (int)roi.y_offset, (int)roi.width, (int)roi.height );
}

// based on https://stackoverflow.com/q/35669182/882436
template <typename PointT>
inline std::pair<PointT, PointT> minmax_3d( const std::vector<PointT>& points ) {

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

}}  // ns
#endif