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
    , TOPIC_NAME_SEGMENT = "segment"
    , TOPIC_NAME_FULL = "full"
    , TOPIC_NAME_FULL_CURRENT = "full_current"
  ;

  // sums the differences in angles for all segments, assumes clockwise rotation/increasing angles
  float sum_of_angles( const std::deque<std::pair<sensor_msgs::PointCloud2, float>>& segments ) {

    float result = 0.f;
    for ( int i = 1; i < segments.size(); ++i ) {
      auto current_angle = segments[i].second;
      auto prev_angle = segments[i-1].second;
      if ( prev_angle > current_angle ) // crossover
        prev_angle -= 360.f;
      result += std::abs( current_angle - prev_angle );  // get diff
    }
    return result;
  }

  sensor_msgs::PointCloud2 radarsegment_to_pointcloud( 
    const asv_perception_common::RadarSegment& segment, 
    const float angle_offset, 
    const std::uint8_t min_intensity,
    const std::string& frame_id
  ) {

    pointcloud_type cloud = {};

    for ( const auto& spoke : segment.spokes ) {
    
        const float
            angle = spoke.angle + angle_offset,  // - 270, // correct for orientation forward
            angle_sin = std::sin(angle * M_PI/180.0),
            angle_cos = std::cos(angle * M_PI/180.0),
            max_range = spoke.max_range
            ;
        const int pixels_in_spoke = spoke.data.size();

        // NODELET_ERROR("spoke angle %s, range %s", std::to_string( angle ).c_str(), std::to_string(max_range).c_str() );

        for ( int i = 0; i < pixels_in_spoke; ++i ) {

            // min intensity check
            if ( spoke.data[i] < min_intensity )
                continue;

            point_type point = {};
            point.x = max_range * float(i)/float(pixels_in_spoke-1) * angle_sin;
            point.y = max_range * float(i)/float(pixels_in_spoke-1) * angle_cos;
            point.z = 0.0;
            cloud.push_back(point);
        } // for
    } // for

    sensor_msgs::PointCloud2 result = {};

    pcl::toROSMsg ( cloud, result );
    result.header = segment.header;
    
    if ( !frame_id.empty() )
      result.header.frame_id = frame_id;

    return result;
  } // radarsegment_to_pointcloud
} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void RadarToPointCloudNodelet::onInit ()
{
  // Call the super onInit ()
  base_type::onInit ();

  // get parameters
  int val = 0;  
  if ( pnh_->getParam("min_intensity", val ) && ( val >= 0 ) )
    this->min_intensity_ = (std::uint8_t)val;

  pnh_->getParam("frame_id", this->frame_id_ );
  pnh_->getParam("angle_offset", this->angle_offset_ );

  // publishers
  this->pub_segment_ = advertise<sensor_msgs::PointCloud2>( *pnh_, TOPIC_NAME_SEGMENT, 1 );
  this->pub_full_ = advertise<sensor_msgs::PointCloud2>( *pnh_, TOPIC_NAME_FULL, 1 );
  this->pub_full_current_ = advertise<sensor_msgs::PointCloud2>( *pnh_, TOPIC_NAME_FULL_CURRENT, 1 );

  onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void RadarToPointCloudNodelet::subscribe ()
{
  lock_type_ lg( this->mtx_ );

  this->sub_ = pnh_->subscribe<asv_perception_common::RadarSegment> (
    TOPIC_NAME_INPUT
    , 1
    , bind (&RadarToPointCloudNodelet::sub_callback, this, _1 )
  );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void RadarToPointCloudNodelet::unsubscribe ()
{
  lock_type_ lg( this->mtx_ );
  this->sub_.shutdown();
}

void RadarToPointCloudNodelet::sub_callback (
      const asv_perception_common::RadarSegment::ConstPtr& segment
)
{
  
  lock_type_ lg( this->mtx_ );

  try {

    assert( segment.get() );
    auto pc_current = ::radarsegment_to_pointcloud( *segment, this->angle_offset_, this->min_intensity_, this->frame_id_ );

    if ( this->pub_segment_.getNumSubscribers() > 0 )
      this->pub_segment_.publish( pc_current );

    // concat needed?
    if ( ( this->pub_full_.getNumSubscribers() > 0 ) || ( this->pub_full_current_.getNumSubscribers() > 0 ) ) {
      
      // remove old segments
      while ( ::sum_of_angles( this->segments_ ) > 360.f )
        this->segments_.pop_front();

      assert( segment->spokes.size() > 0 );

      const auto current_angle = std::abs(segment->spokes[0].angle);
      this->segments_.emplace_back( std::move(pc_current), current_angle ); // pc_current moved

      // are we at crossover point?  assumes input radar segments are clockwise
      const bool at_crossover = ( ( this->segments_.size() > 1 ) && ( ( current_angle - this->segments_[this->segments_.size()-2].second ) < 0.f ) );

      // pointcloud concat
      sensor_msgs::PointCloud2 result = {};
      for ( auto& pc_pair : this->segments_ ) {
        auto& pc = pc_pair.first;
        if ( ( pc.width > 0 ) && !pcl::concatenatePointCloud( result, pc, result ) )
          ROS_ERROR("Error concatenating point clouds");
      }
      
      // use latest header
      result.header = this->segments_.back().first.header;

      if ( this->pub_full_current_.getNumSubscribers() > 0 )
        this->pub_full_current_.publish( result );
      
      // print 'full' if at crossover point
      if ( at_crossover && ( this->pub_full_.getNumSubscribers() > 0 ) )
        this->pub_full_.publish( result );

    }

  } catch ( const std::exception& ex ) {  // pcl exceptions inherit from std::runtime_error
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }
}


PLUGINLIB_EXPORT_CLASS( obstacle_id::RadarToPointCloudNodelet, nodelet::Nodelet)