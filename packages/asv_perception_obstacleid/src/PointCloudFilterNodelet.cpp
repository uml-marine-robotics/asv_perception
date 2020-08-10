#include "PointCloudFilterNodelet.h"
#include <math.h> // pow

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include "utils.h"
#include "detail/PointCluster.h"

namespace {
  using namespace obstacle_id;

  static const std::string 
    TOPIC_NAME_INPUT = "input"
    , TOPIC_NAME_OUTPUT = "output"
  ;
} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudFilterNodelet::onInit ()
{
  // Call the super onInit ()
  base_type::onInit ();

  // parameters
  pnh_->getParam("min_distance", this->min_distance_ );
  pnh_->getParam("min_distance_x", this->min_distance_x_ );
  pnh_->getParam("min_distance_y", this->min_distance_y_ );
  pnh_->getParam("min_distance_z", this->min_distance_z_ );
  pnh_->getParam( "outlier_min_neighbors", this->outlier_min_neighbors_ );
  pnh_->getParam( "outlier_radius", this->outlier_radius_ );

  int val = 0;  
  if ( pnh_->getParam("cluster_size_max", val ) && ( val >= 0 ) )
    this->cluster_sz_max_ = (std::uint32_t)val;

  if ( pnh_->getParam("cluster_size_min", val ) && ( val >= 0 ) )
    this->cluster_sz_min_ = (std::uint32_t)val;

  pnh_->getParam("cluster_tolerance", this->cluster_tolerance_ );
  pnh_->getParam("cluster_area_max", this->cluster_area_max_ );
  pnh_->getParam("cluster_area_min", this->cluster_area_min_ );
  pnh_->getParam("cluster_inliers", this->cluster_inliers_ );

  // publisher
  this->pub_ = advertise<sensor_msgs::PointCloud2>( *pnh_, TOPIC_NAME_OUTPUT, 1 );

  onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudFilterNodelet::subscribe ()
{
  this->sub_ = pnh_->subscribe<sensor_msgs::PointCloud2> (
    TOPIC_NAME_INPUT
    , 1
    , bind (&PointCloudFilterNodelet::sub_callback, this, _1 )
  );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudFilterNodelet::unsubscribe ()
{
  this->sub_.shutdown();
}

void PointCloudFilterNodelet::sub_callback (
      const sensor_msgs::PointCloud2::ConstPtr& cloud
)
{
  using point_type = pcl::PointXYZ;
  using pointcloud_type = pcl::PointCloud<point_type>;

  // No subscribers, no work
  if ( this->pub_.getNumSubscribers () < 1 )
    return;

  // empty cloud, no work
  if ( cloud->data.empty() )
    return;

  try {
    pointcloud_type::Ptr pc_ptr(new pointcloud_type());
    pcl::fromROSMsg( *cloud, *pc_ptr );

    if ( pc_ptr->empty() )
      return;

    // clustering filter
    if ( this->cluster_tolerance_ > 0.f ) {

        pointcloud_type::Ptr filtered( new pointcloud_type{} );
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<point_type> extract = {};

        for ( const auto& cluster : detail::PointCluster::extract( 
            pc_ptr
            , this->cluster_tolerance_
            , this->cluster_sz_min_
            , this->cluster_sz_max_
            , this->cluster_area_min_
            , this->cluster_area_max_
            ) ) {

            for ( const auto idx : cluster.indices.indices )
              inliers->indices.push_back(idx);
        }

        extract.setInputCloud( pc_ptr );
        extract.setIndices(inliers);
        extract.setNegative( !this->cluster_inliers_ );  // setNegative(true) = remove the inliers
        extract.filter(*pc_ptr);
    }

    // min distance filter (radius from origin)
    if ( !pc_ptr->empty() && ( this->min_distance_ > 0.f ) ) {

        const auto d_2 = std::pow( this->min_distance_, 2. );

        // https://stackoverflow.com/a/48595186/882436
        pointcloud_type::Ptr filtered( new pointcloud_type{} );
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<point_type> extract = {};
        for ( std::size_t i = 0; i < pc_ptr->points.size(); ++i ) {
          const auto& pt = pc_ptr->points[i];
          if ( ( std::pow(pt.x,2.) + std::pow(pt.y,2.) + std::pow(pt.z,2.) ) < d_2 )
            inliers->indices.push_back(i);
        }
        extract.setInputCloud( pc_ptr );
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*pc_ptr);
    }

    // dimensional min distance filter
    if ( !pc_ptr->empty() 
        &&
        ( ( this->min_distance_x_ > 0.f ) || ( this->min_distance_y_ > 0.f ) || ( this->min_distance_z_ > 0.f ) ) 
    ) {
        using PointType = point_type;
        pcl::ConditionOr<PointType>::Ptr range_cond (new pcl::ConditionOr<PointType> ());
        
        if ( this->min_distance_x_ > 0.f ) {
            range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
                                            pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::GT, this->min_distance_x_ )));
            range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
                                            pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::LT, -this->min_distance_x_ )));
        }

        if ( this->min_distance_y_ > 0.f ) {
            range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
                                            pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::GT, this->min_distance_y_ )));
            range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
                                            pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::LT, -this->min_distance_y_ )));
        }

        if ( this->min_distance_z_ > 0.f ) {
            range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
                                            pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::GT, this->min_distance_z_ )));
            range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
                                            pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::LT, -this->min_distance_z_ )));
        }

        pointcloud_type::Ptr cloud_post_filter (new pointcloud_type);
        pcl::ConditionalRemoval<PointType> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(pc_ptr);
        condrem.filter(*cloud_post_filter);
        pc_ptr = cloud_post_filter;
    }

    // radius outlier removal
    if ( !pc_ptr->empty() && ( this->outlier_min_neighbors_ > 0 ) && ( this->outlier_radius_ > 0. ) ) {
        pcl::RadiusOutlierRemoval<point_type> outrem = {};
        outrem.setInputCloud( pc_ptr );
        outrem.setRadiusSearch( this->outlier_radius_ );
        outrem.setMinNeighborsInRadius( this->outlier_min_neighbors_ );
        outrem.filter (*pc_ptr );
    }
    
    // publish
    this->pub_.publish( pc_ptr );

  } catch ( const std::exception& ex ) {  // pcl exceptions inherit from std::runtime_error
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }
}


PLUGINLIB_EXPORT_CLASS( obstacle_id::PointCloudFilterNodelet, nodelet::Nodelet)