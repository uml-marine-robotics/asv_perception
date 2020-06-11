#ifndef OBSTACLEID_UTILS_H
#define OBSTACLEID_UTILS_H

#include <sensor_msgs/RegionOfInterest.h>
#include <geometry_msgs/Point.h>
#include "defs.h"

namespace obstacle_id {
namespace utils {

// convert ros roi to opencv rect
inline cv::Rect to_cv_rect( const sensor_msgs::RegionOfInterest& roi ) {
    return cv::Rect( (int)roi.x_offset, (int)roi.y_offset, (int)roi.width, (int)roi.height );
}

}}  // ns
#endif