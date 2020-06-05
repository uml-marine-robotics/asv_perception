#ifndef CONVEX_HULL_NODELET_H
#define CONVEX_HULL_NODELET_H

#include <pcl_ros/pcl_nodelet.h>
#include <ros/ros.h>
#include "defs.h"

namespace obstacle_id
{
  /*
  Nodelet for visualization of pointclouds in rviz
  Topics:
  - input:  input cloud
  - labels:  [todo] labels from uint32_t classid --> string
  - output:  MarkerArray
  */
  class ClusterVisualizationNodelet 
    : public pcl_ros::PCLNodelet
  {
    public:
      
      // default constructor
      ClusterVisualizationNodelet() = default;
                                      
    protected:

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

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 