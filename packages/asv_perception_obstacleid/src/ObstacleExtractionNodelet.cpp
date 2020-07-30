
#include "ObstacleExtractionNodelet.h"

#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <asv_perception_common/ObstacleArray.h>
// #include <pcl/io/pcd_io.h>  // debug

#include "utils.h"
#include "detail/obstacle_extraction.h"

namespace {
  using namespace obstacle_id;

  static const std::string 
    TOPIC_NAME_INPUT = "input"
    , TOPIC_NAME_OUTPUT = "obstacles"
  ;
} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleExtractionNodelet::onInit ()
{
  // Call the super onInit ()
  base_type::onInit ();

  // Mandatory parameters
  if ( !pnh_->getParam("cluster_tolerance", this->cluster_tolerance_ ))
  {
    NODELET_ERROR ("[%s::onInit] Need a 'cluster_tolerance' parameter to be set before continuing!", getName ().c_str ()); 
    return;
  }

  // optional parameters
  int val = 0;  
  if ( pnh_->getParam("max_cluster_size", val ) && ( val >= 0 ) )
    this->max_cluster_sz_ = (std::uint32_t)val;

  if ( pnh_->getParam("min_cluster_size", val ) && ( val >= 0 ) )
    this->min_cluster_sz_ = (std::uint32_t)val;

  pnh_->getParam("max_area", this->max_area_);

  // publisher
  this->pub_ = advertise<asv_perception_common::ObstacleArray>( *pnh_, TOPIC_NAME_OUTPUT, 1 );

  onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleExtractionNodelet::subscribe ()
{
  this->sub_ = pnh_->subscribe<sensor_msgs::PointCloud2> (
    TOPIC_NAME_INPUT
    , 1
    , bind (&ObstacleExtractionNodelet::sub_callback, this, _1 )
  );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleExtractionNodelet::unsubscribe ()
{
  this->sub_.shutdown();
}

void ObstacleExtractionNodelet::sub_callback (
      const sensor_msgs::PointCloud2::ConstPtr& cloud
)
{
  // No subscribers, no work
  if ( this->pub_.getNumSubscribers () < 1 )
    return;

  // If cloud is given, check if it's valid
  if ( !utils::is_cloud_valid( cloud ) )
  {
    // NODELET_ERROR ("[%s::sub_callback] Invalid input cloud!", getName ().c_str ());
    ROS_ERROR("Invalid cloud");
    return;
  }

  // empty cloud, no work
  if ( cloud->data.empty() )
    return;

  try {
    pointcloud_type::Ptr pc_ptr(new pointcloud_type());
    pcl::fromROSMsg( *cloud, *pc_ptr );

    if ( pc_ptr->empty() )
      return;

    // debug
    // pcl::io::savePCDFile( "/tmpdata/cloud.pcd", *pc_ptr, true ); // Binary format

    // perform obstacle extraction
    asv_perception_common::ObstacleArray msg = {};
    msg.header = cloud->header;

    msg.obstacles = detail::obstacle_extraction::extract( 
      pc_ptr, this->cluster_tolerance_, this->min_cluster_sz_, this->max_cluster_sz_, this->max_area_
      );

    // set header for obstacles
    for ( auto& obs : msg.obstacles )
      obs.header = cloud->header;

    this->pub_.publish( msg );

  } catch ( const std::exception& ex ) {  // pcl exceptions inherit from std::runtime_error
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }


}


PLUGINLIB_EXPORT_CLASS(obstacle_id::ObstacleExtractionNodelet, nodelet::Nodelet)