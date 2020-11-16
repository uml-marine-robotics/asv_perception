// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

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
  pnh_->getParam("decay_time", this->decay_time_ );

  assert( this->decay_time_ > 0. );

  // publishers
  this->pub_full_ = advertise<sensor_msgs::PointCloud2>( *pnh_, TOPIC_NAME_OUTPUT_FULL, 1 );
  this->pub_current_ = advertise<sensor_msgs::PointCloud2>( *pnh_, TOPIC_NAME_OUTPUT_CURRENT, 1 );

  onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatNodelet::subscribe ()
{
  lock_type_ lg( this->mtx_ );

  this->last_full_publish_=ros::Time::now();

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

  const auto now = ros::Time::now();

  // cleanup old segments
  std::vector<std::pair<sensor_msgs::PointCloud2::ConstPtr, ros::Time>> valid = {};

  for ( auto& seg : this->segments_ ) {
    
    const auto diff = std::abs( ( now - seg.second ).toSec() ); // using abs in case of time jumps due to rosbag restart

    if ( diff <= this->decay_time_ )
      valid.emplace_back(std::move(seg));
  }

  valid.emplace_back( cloud, now ); // use msg header time?
  this->segments_ = std::move( valid );

  const bool time_for_full_publish = ( ( now - this->last_full_publish_ ).toSec() > this->decay_time_ );

  // concat and publish when we have a subscriber to 'current', or when we're due for a full publish
  if ( ( this->pub_current_.getNumSubscribers() < 1 ) && !time_for_full_publish )
    return;

  try {
    /*
     https://github.com/ros-perception/perception_pcl/blob/b1917efe78112300473590f25e93fd8edbdac7c4/pcl_conversions/include/pcl_conversions/pcl_conversions.h#L611
     pcl_conversions:  bool concatenatePointCloud (const sensor_msgs::PointCloud2 &cloud1,
                              const sensor_msgs::PointCloud2 &cloud2,
                              sensor_msgs::PointCloud2 &cloud_out)
    */

    sensor_msgs::PointCloud2 result = {};
    for ( auto& pc_pair : this->segments_ ) {
      auto& pc = pc_pair.first;
      if ( pc != nullptr && ( pc->width > 0 ) && !pcl::concatenatePointCloud( result, *pc, result ) )
        ROS_ERROR("Error concatenating point clouds");
    }
    
    // use latest header
    result.header = cloud->header;

    if ( this->pub_current_.getNumSubscribers() > 0 )
      this->pub_current_.publish( result );

    if ( time_for_full_publish && ( this->pub_full_.getNumSubscribers() > 0 ) ) {
      this->pub_full_.publish( result );
      this->last_full_publish_=now;
    }

  } catch ( const std::exception& ex ) {  // pcl exceptions inherit from std::runtime_error
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }
}


PLUGINLIB_EXPORT_CLASS( obstacle_id::PointCloudConcatNodelet, nodelet::Nodelet)