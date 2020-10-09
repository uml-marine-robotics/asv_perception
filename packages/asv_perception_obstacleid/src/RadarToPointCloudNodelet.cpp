#include "RadarToPointCloudNodelet.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>

#include "defs.h"

namespace {
  using namespace obstacle_id;

  static const std::string 
    TOPIC_NAME_INPUT = "input"
    , TOPIC_NAME_OUTPUT = "output"
  ;
} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void RadarToPointCloudNodelet::onInit ()
{
  // Call the super onInit ()
  base_type::onInit ();

  // publisher
  this->pub_ = advertise<sensor_msgs::PointCloud2>( *pnh_, TOPIC_NAME_OUTPUT, 1 );

  onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void RadarToPointCloudNodelet::subscribe ()
{
  this->sub_ = pnh_->subscribe<asv_perception_common::RadarSegment> (
    TOPIC_NAME_INPUT
    , 1
    , bind (&RadarToPointCloudNodelet::sub_callback, this, _1 )
  );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void RadarToPointCloudNodelet::unsubscribe ()
{
  this->sub_.shutdown();
}

void RadarToPointCloudNodelet::sub_callback (
      const asv_perception_common::RadarSegment::ConstPtr& segment
)
{
  // No subscribers, no work
  if ( this->pub_.getNumSubscribers () < 1 )
    return;

  try {
    pointcloud_type::Ptr cloud(new pointcloud_type());
    cloud->header.frame_id = "velodyne";
    pcl_conversions::toPCL(segment->header.stamp, cloud->header.stamp);

    for ( const auto& spoke : segment->spokes ) {
    
        const float
            angle = spoke.angle - 270, // correct for orientation forward
            angle_sin = std::sin(angle * M_PI/180.0),
            angle_cos = std::cos(angle * M_PI/180.0),
            max_range = spoke.max_range
            ;
        const int pixels_in_spoke = spoke.data.size();

        // NODELET_ERROR("spoke angle %s, range %s", std::to_string( angle ).c_str(), std::to_string(max_range).c_str() );

        for ( int i = 0; i < pixels_in_spoke; ++i ) {

            // don't publish 0-intensity points
            if ( spoke.data[i] == 0 )
                continue;

            point_type point = {};
            point.x = max_range * float(i)/float(pixels_in_spoke-1) * angle_sin;
            point.y = max_range * float(i)/float(pixels_in_spoke-1) * angle_cos;
            point.z = 0.0;
            cloud->push_back(point);
        }
    }
    
    // publish
    this->pub_.publish( cloud );

  } catch ( const std::exception& ex ) {  // pcl exceptions inherit from std::runtime_error
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }
}


PLUGINLIB_EXPORT_CLASS( obstacle_id::RadarToPointCloudNodelet, nodelet::Nodelet)