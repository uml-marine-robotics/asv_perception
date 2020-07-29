#include "PointCloudConcatNodelet.h"

#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>

namespace {
  using namespace obstacle_id;

  static const std::string 
    TOPIC_NAME_INPUT = "input"
    , TOPIC_NAME_OUTPUT_FULL = "full"
    , TOPIC_NAME_OUTPUT_CURRENT = "current"
  ;
} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatNodelet::onInit ()
{
  // Call the super onInit ()
  base_type::onInit ();

  // parameters
  pnh_->getParam("segments", this->nsegments_ );

  assert( this->nsegments_ > 0 );
  this->segments_.resize( this->nsegments_ );

  // publishers
  this->pub_full_ = advertise<sensor_msgs::PointCloud2>( *pnh_, TOPIC_NAME_OUTPUT_FULL, 1 );
  this->pub_current_ = advertise<sensor_msgs::PointCloud2>( *pnh_, TOPIC_NAME_OUTPUT_CURRENT, 1 );

  onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatNodelet::subscribe ()
{
  lock_type_ lg( this->mtx_ );

  this->segments_.clear();
  this->segments_.resize( this->nsegments_ );
  this->current_idx_ = 0;

  this->sub_ = pnh_->subscribe<sensor_msgs::PointCloud2> (
    TOPIC_NAME_INPUT
    , 1
    , bind (&PointCloudConcatNodelet::sub_callback, this, _1 )
  );

}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatNodelet::unsubscribe ()
{
  lock_type_ lg( this->mtx_ );

  this->sub_.shutdown();
}

void PointCloudConcatNodelet::sub_callback (
      const sensor_msgs::PointCloud2::ConstPtr& cloud
)
{
  lock_type_ lg( this->mtx_ );

  this->segments_[this->current_idx_] = cloud;
  this->current_idx_ = ++this->current_idx_ % this->nsegments_;

  // concat and publish when we have a subscriber to 'current', or when current_idx == 0
  if ( this->pub_current_.getNumSubscribers() < 1 && this->current_idx_ != 0 )
    return;

  try {
    /*
     https://github.com/ros-perception/perception_pcl/blob/b1917efe78112300473590f25e93fd8edbdac7c4/pcl_conversions/include/pcl_conversions/pcl_conversions.h#L611
     pcl_conversions:  bool concatenatePointCloud (const sensor_msgs::PointCloud2 &cloud1,
                              const sensor_msgs::PointCloud2 &cloud2,
                              sensor_msgs::PointCloud2 &cloud_out)
    */

    sensor_msgs::PointCloud2 result = {};
    for ( auto& pc : this->segments_ )
      if ( pc != nullptr && !pcl::concatenatePointCloud( result, *pc, result ) )
        ROS_ERROR("Error concatenating point clouds");
    
    // use latest header
    result.header = cloud->header;

    if ( this->pub_current_.getNumSubscribers() > 0 )
      this->pub_current_.publish( result );

    if ( this->pub_full_.getNumSubscribers() > 0 )
      this->pub_full_.publish( result );

  } catch ( const std::exception& ex ) {  // pcl exceptions inherit from std::runtime_error
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }
}


PLUGINLIB_EXPORT_CLASS( obstacle_id::PointCloudConcatNodelet, nodelet::Nodelet)