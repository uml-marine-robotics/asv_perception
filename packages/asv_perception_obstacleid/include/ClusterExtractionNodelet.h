#ifndef CLUSTER_EXTRACTION_NODELET_H
#define CLUSTER_EXTRACTION_NODELET_H

#include <pcl_ros/pcl_nodelet.h>
#include <ros/ros.h>
#include "defs.h"

namespace obstacle_id
{
  /*
  Cluster extraction nodelet is a ROS nodelet which takes a pointcloud of a labeled points and extracts labeled clusters
  Based on pcl_ros nodelet, https://github.com/ros-perception/perception_pcl/blob/melodic-devel/pcl_ros/include/pcl_ros/segmentation/extract_clusters.h
  Parameters:
  - cluster_tolerance: (required)  clustering tolerance, type=float
  - max_clusters:   maximum number of clusters to return, type=uint, default=(max value)
  - min_cluster_size:  minimum number of points in a cluster.  type=uint, default=1
  - max_cluster_size:  maximum number of points in a cluster.  type=uint, default=(max value)
  Topics:
  - input:  input labeled pointcloud
  - output:  output labeled pointclouds, one per labeled cluster
  - convex_hull_2d:  output 2d labeled pointclouds (where z==0), one per labeled cluster
  - descriptor:  output string containing obstacle info (TBD)
  */
  class ClusterExtractionNodelet 
    : public pcl_ros::PCLNodelet
  {
    public:
      
      // default constructor
      ClusterExtractionNodelet() = default;
                                      
    protected:

      /** \brief Maximum number of clusters to publish. */
      std::uint32_t _max_clusters = std::numeric_limits<std::uint32_t>::max();

      // Minimum cluster size
      std::uint32_t _min_cluster_sz = 1;

      // Maximum cluster size
      std::uint32_t _max_cluster_sz = std::numeric_limits<std::uint32_t>::max();

      // Cluster tolerance
      float _cluster_tolerance = 0.f;

      /** \brief Nodelet initialization routine. */
      void onInit () override;

      /** \brief LazyNodelet connection routine. */
      void subscribe () override;
      void unsubscribe () override;

      // the callback function to handle input from subscription
      void sub_callback ( const sensor_msgs::PointCloud2::ConstPtr& );
      
    private:

      /** \brief The input PointCloud subscriber. */
      ros::Subscriber _sub_input;

      
      ros::Publisher
        // 2d convex hull
        _pub_convex_hull_2d
        // string descriptor
        , _pub_descriptor
        ;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 