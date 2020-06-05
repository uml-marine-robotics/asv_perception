
#include "ClusterExtractionNodelet.h"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include <pcl/common/common.h>      // getMinMax3D
#include <pcl/PointIndices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_labeled_clusters.h>
#include <pcl/surface/convex_hull.h>    // convexHull

#include <pcl_conversions/pcl_conversions.h>

namespace {
  using namespace obstacle_id;

  static const std::string 
    TOPIC_NAME_INPUT = "input"
    , TOPIC_NAME_OUTPUT = "output"
    , TOPIC_NAME_OUTPUT_DESCRIPTOR = "descriptor"
    , TOPIC_NAME_OUTPUT_CONVEX_HULL_2D = "convex_hull_2d"
  ;

  std::string create_obstacle_descriptor( const pointcloud_type& pc ) {

      point_type min_pt, max_pt;
      pcl::getMinMax3D(pc, min_pt, max_pt);
      
      const auto& first_pt = pc.points.front();
      auto result = std::to_string(first_pt.label);  // TODO:  labelToString( first_pt.label );

      const auto fmt_pt = []( const point_type& pt ) {
          return std::string("x=")
              + std::to_string( (int) pt.x)
              + ",y="
              + std::to_string( (int)pt.y )
              + ",z=" + std::to_string( (int)pt.z )
              ;
      };

      return result + ";" + fmt_pt( min_pt ) + ";" + fmt_pt(max_pt);
  }
} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void ClusterExtractionNodelet::onInit ()
{
  // Call the super onInit ()
  PCLNodelet::onInit ();

  // Mandatory parameters
  if ( !pnh_->getParam ("cluster_tolerance", this->_cluster_tolerance ))
  {
    NODELET_ERROR ("[%s::onInit] Need a 'cluster_tolerance' parameter to be set before continuing!", getName ().c_str ()); 
    return;
  }

  // optional parameters
  int val = 0;
  if ( pnh_->getParam("max_clusters", val ) && val >= 0 )
    this->_max_clusters = (std::uint32_t)val;
  
  if ( pnh_->getParam("max_cluster_size", val ) && val >= 0 )
    this->_max_cluster_sz = (std::uint32_t)val;

  if ( pnh_->getParam("min_cluster_size", val ) && val >= 0 )
    this->_min_cluster_sz = (std::uint32_t)val;

  this->pub_output_ = advertise<sensor_msgs::PointCloud2> (*pnh_, TOPIC_NAME_OUTPUT, this->max_queue_size_);
  this->_pub_convex_hull_2d = advertise<sensor_msgs::PointCloud2> (*pnh_, TOPIC_NAME_OUTPUT_CONVEX_HULL_2D, this->max_queue_size_);
  this->_pub_descriptor = advertise<std_msgs::String> (*pnh_, TOPIC_NAME_OUTPUT_DESCRIPTOR, this->max_queue_size_);

  NODELET_DEBUG("[%s::onInit] Initializing node with parameters: cluster_tolerance=%f, max_clusters=%u, max_cluster_size=%u, min_cluster_size=%u"
    , getName ().c_str()
    , this->_cluster_tolerance
    , this->_max_clusters
    , this->_max_cluster_sz
    , this->_min_cluster_sz
  );

  onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ClusterExtractionNodelet::subscribe ()
{
  this->_sub_input = pnh_->subscribe<sensor_msgs::PointCloud2> (
    TOPIC_NAME_INPUT
    , this->max_queue_size_
    , bind (&ClusterExtractionNodelet::sub_callback, this, _1 )
  );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ClusterExtractionNodelet::unsubscribe ()
{
  this->_sub_input.shutdown();
}


void ClusterExtractionNodelet::sub_callback (
      const sensor_msgs::PointCloud2::ConstPtr& cloud
)
{
  // No subscribers, no work
  if ( 
    this->pub_output_.getNumSubscribers () <= 0 
    && this->_pub_convex_hull_2d.getNumSubscribers() <= 0 
    && this->_pub_descriptor.getNumSubscribers() <= 0
    )
    return;

  // If cloud is given, check if it's valid
  if (!isValid (cloud))
  {
    NODELET_ERROR ("[%s::sub_callback] Invalid input!", getName ().c_str ());
    return;
  }

  // empty cloud, no work
  if ( cloud->data.empty() )
    return;

  pointcloud_type::Ptr pc(new pointcloud_type());
  pcl::fromROSMsg( *cloud, *pc );

  // labeled euclidean cluster extraction
  //  clusters array must have preallocated size > max_label, else segfault
  //  https://github.com/PointCloudLibrary/pcl/pull/4084
  //  alternative:  ConditionalEuclideanClustering
  std::vector< std::vector< pcl::PointIndices> > labeled_clusters = {};
  
  const auto max_label_it = std::max_element( pc->points.begin(), pc->points.end()
    , [](const point_type& a, const point_type& b) { return a.label < b.label; }
  );

  assert(max_label_it != pc->points.end() );
  labeled_clusters.resize(max_label_it->label + 1);
  pcl::search::Search<point_type>::Ptr tree2( new pcl::search::KdTree<point_type>);
  tree2->setInputCloud(pc);

  pcl::extractLabeledEuclideanClusters( *pc, tree2, this->_cluster_tolerance, labeled_clusters, this->_min_cluster_sz, this->_max_cluster_sz );

  std::size_t nclusters = 0;
  bool reached_max_clusters = false;

  // output the clusters
  //  labeled clusters is 2d array
  //  first dimension is grouping by label
  //  second dimension is grouping by cluster
  //  outputting the indices would be more efficient, but then the client needs to sync the original cloud along with the indices
  for ( const auto& label : labeled_clusters ) {

    if ( reached_max_clusters ) // done?
      break;

    if ( label.empty() )  // no clusters for this label?
      continue;

    for ( const auto& cluster : label ) { // cluster is a pcl::PointIndices
      
      if ( (nclusters++) >= this->_max_clusters ) {
        reached_max_clusters = true;
        break;
      }

      ROS_ASSERT( !cluster.indices.empty() );

      // generate a new pointcloud based on indices in this cluster
      pointcloud_type::Ptr cluster_pc( new pointcloud_type() );
      pcl::copyPointCloud (*pc, cluster.indices, *cluster_pc);

      // full cluster publishing
      if ( this->pub_output_.getNumSubscribers () > 0 ) {

        NODELET_DEBUG("[%s::sub_callback] Publishing cluster=%u, label=%u, npoints=%u"
          , getName ().c_str()
          , (std::uint32_t)nclusters
          , (std::uint32_t)cluster_pc->points.front().label
          , (std::uint32_t)cluster.indices.size()
          );
        
        sensor_msgs::PointCloud2::Ptr output_blob( new sensor_msgs::PointCloud2() );
        pcl::toROSMsg ( *cluster_pc, *output_blob );

        // publish
        pub_output_.publish ( output_blob );
      }

      // descriptor publishing
      if ( this->_pub_descriptor.getNumSubscribers() > 0 ) {
  
        auto obstacle_msg = create_obstacle_descriptor( *cluster_pc );
        NODELET_DEBUG("[%s::sub_callback] Publishing obstacle message:  %s"
            , getName ().c_str()
            , obstacle_msg.c_str()
        );
        std_msgs::String::Ptr msg (new std_msgs::String());
        msg->data = std::move(obstacle_msg);
        this->_pub_descriptor.publish( msg );
      }

      // convex hull publishing
      //  this will modify points in cluster_pc
      if ( this->_pub_convex_hull_2d.getNumSubscribers() > 0 ) {

        // flatten the plane
        // set all input points to z=0 prior to convex hull generation
        //  any reason to do it the "proper" way here?
        for ( auto& pt : cluster_pc->points )
            pt.z = 0.f;

        pcl::ConvexHull<point_type> chull;
        chull.setInputCloud ( cluster_pc );

        pointcloud_type pc_hull = {};
        chull.reconstruct (pc_hull);

        ROS_ASSERT( !pc_hull.empty() );

        NODELET_DEBUG("[%s::sub_callback] Publishing convex hull for cluster=%u, label=%u, npoints=%u"
          , getName ().c_str()
          , (std::uint32_t)nclusters
          , (std::uint32_t)pc_hull.points.front().label
          , (std::uint32_t)pc_hull.points.size()
          );

        sensor_msgs::PointCloud2::Ptr output_blob(new sensor_msgs::PointCloud2());
        pcl::toROSMsg ( pc_hull, *output_blob );
        this->_pub_convex_hull_2d.publish( output_blob );
      }

    } // for label

  } // for labeled_clusters

}


PLUGINLIB_EXPORT_CLASS(obstacle_id::ClusterExtractionNodelet, nodelet::Nodelet)