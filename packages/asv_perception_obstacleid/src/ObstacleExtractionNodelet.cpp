
#include "ObstacleExtractionNodelet.h"

#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <asv_perception_common/ObstacleArray.h>
// #include <pcl/io/pcd_io.h>  // debug

#include "utils.h"
#include "detail/PointCluster.h"

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
    NODELET_ERROR ("[%s::onInit] Need a 'cluster_tolerance' parameter to be set before continuing!", getName().c_str ()); 
    return;
  }

  // optional parameters
  int val = 0;  
  if ( pnh_->getParam("cluster_size_max", val ) && ( val >= 0 ) )
    this->cluster_sz_max_ = (std::uint32_t)val;

  if ( pnh_->getParam("cluster_size_min", val ) && ( val >= 0 ) )
    this->cluster_sz_min_ = (std::uint32_t)val;

  pnh_->getParam("cluster_area_max", this->cluster_area_max_ );
  pnh_->getParam("cluster_area_min", this->cluster_area_min_ );

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

    // extract clusters from pointcloud
    const auto pclusters = detail::PointCluster::extract( 
      pc_ptr
      , this->cluster_tolerance_
      , this->cluster_sz_min_, this->cluster_sz_max_
      , this->cluster_area_min_, this->cluster_area_max_
    );

    // convert clusters to obstacles
    for ( const auto& pcluster : pclusters )
        msg.obstacles.emplace_back( pcluster.to_obstacle() );

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