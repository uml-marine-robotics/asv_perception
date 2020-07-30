#ifndef OBSTACLE_EXTRACTION_NODELET_H
#define OBSTACLE_EXTRACTION_NODELET_H

#include <pcl_ros/pcl_nodelet.h>
#include <ros/ros.h>
#include "defs.h"

namespace obstacle_id
{
  /*
  Obstacle extraction nodelet performs euclidean clustering and constructs Obstacle messages

  Parameters:
    ~cluster_tolerance: [float]                    clustering tolerance
    ~min_cluster_size:  [uint, default=1]          minimum number of points in a cluster
    ~max_cluster_size:  [uint, default=uint_max]   maximum number of points in a cluster
    ~max_area:          [float, default=0]         maximum convex hull area
  
  Topics:
    ~input:      [sensor_msgs/PointCloud2]               input pointcloud
    ~obstacles:  [asv_perception_common/ObstacleArray]   output obstacles
  */
  class ObstacleExtractionNodelet 
    : public nodelet_topic_tools::NodeletLazy
  {
    public:

      using base_type = nodelet_topic_tools::NodeletLazy;
      
      // default constructor
      ObstacleExtractionNodelet() = default;
                                      
    protected:

      std::uint32_t 
        min_cluster_sz_ = 1
        , max_cluster_sz_ = std::numeric_limits<std::uint32_t>::max()
        ;

      float 
        cluster_tolerance_ = 0.f
        , max_area_ = 0.f
        ;

      /** \brief Nodelet initialization routine. */
      void onInit () override;

      /** \brief LazyNodelet connection routine. */
      void subscribe () override;
      void unsubscribe () override;

      // the callback function to handle input from subscription
      void sub_callback ( const sensor_msgs::PointCloud2::ConstPtr& );
      
    private:

      ros::Subscriber sub_;
      ros::Publisher pub_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 