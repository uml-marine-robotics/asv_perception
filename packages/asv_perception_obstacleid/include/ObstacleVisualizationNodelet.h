#ifndef OBSTACLE_VIS_NODELET_H
#define OBSTACLE_VIS_NODELET_H

#include <pcl_ros/pcl_nodelet.h>
#include <ros/ros.h>
#include <asv_perception_common/ObstacleArray.h>
#include "defs.h"

namespace obstacle_id
{
  /*
  Nodelet for visualization of pointclouds in rviz
  Topics:
  - input:  [asv_perception_common/ObstacleArray.msg]
  - output:  MarkerArray
  */
  class ObstacleVisualizationNodelet 
    : public pcl_ros::PCLNodelet
  {
    public:
      
      // default constructor
      ObstacleVisualizationNodelet() = default;
                                      
    protected:

      /** \brief Nodelet initialization routine. */
      void onInit () override;

      /** \brief LazyNodelet connection routine. */
      void subscribe () override;
      void unsubscribe () override;

      // the callback function to handle input from subscription
      void sub_callback ( typename asv_perception_common::ObstacleArray::ConstPtr );
      
    private:

      /** \brief The input subscriber. */
      ros::Subscriber _sub_input;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 