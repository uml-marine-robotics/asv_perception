#ifndef OBSTACLE_VIS_NODELET_H
#define OBSTACLE_VIS_NODELET_H

#include <ros/ros.h>
#include <nodelet_topic_tools/nodelet_lazy.h>
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
    : public nodelet_topic_tools::NodeletLazy
  {
    public:
      
      using base_type = nodelet_topic_tools::NodeletLazy;
      
      // default constructor
      ObstacleVisualizationNodelet() = default;
                                      
    protected:

      /** \brief Nodelet initialization routine. */
      void onInit () override;

      /** \brief LazyNodelet connection routine. */
      void subscribe () override;
      void unsubscribe () override;

      // the callback function to handle input from subscription
      void sub_callback ( const typename asv_perception_common::ObstacleArray::ConstPtr& );
      
    private:
      // ObstacleArray
      ros::Subscriber _in;

      // markerarray
      ros::Publisher _out;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 