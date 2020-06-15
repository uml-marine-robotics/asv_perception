#ifndef OBSTACLE_EXTRACTION_NODELET_H
#define OBSTACLE_EXTRACTION_NODELET_H

#include <pcl_ros/pcl_nodelet.h>
#include <ros/ros.h>
#include "defs.h"

namespace obstacle_id
{
  /*
  Obstacle extraction nodelet is a ROS nodelet which takes in a pointcloud extracts obstacles
  Performs euclidean clustering and constructs Obstacle messages

  Based on pcl_ros nodelet, https://github.com/ros-perception/perception_pcl/blob/melodic-devel/pcl_ros/include/pcl_ros/segmentation/extract_clusters.h

  Parameters:
  - cluster_tolerance: (required)  clustering tolerance, type=float
  - max_clusters:   maximum number of clusters to return, type=uint, default=(max value)
  - min_cluster_size:  minimum number of points in a cluster.  type=uint, default=1
  - max_cluster_size:  maximum number of points in a cluster.  type=uint, default=(max value)
  Topics:
  - cloud:  input pointcloud
  - obstacles:  output obstacles
  */
  class ObstacleExtractionNodelet 
    : public nodelet_topic_tools::NodeletLazy
  {
    public:

      using base_type = nodelet_topic_tools::NodeletLazy;
      
      // default constructor
      ObstacleExtractionNodelet() = default;
                                      
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
      ros::Subscriber _sub;
      
      ros::Publisher
        _pub
        ;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 