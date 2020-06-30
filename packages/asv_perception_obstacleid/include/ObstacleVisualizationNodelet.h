#ifndef OBSTACLE_VIS_NODELET_H
#define OBSTACLE_VIS_NODELET_H

#include <ros/ros.h>
#include <nodelet_topic_tools/nodelet_lazy.h>
#include <asv_perception_common/ObstacleArray.h>
#include "defs.h"

namespace obstacle_id
{
  /*
  Nodelet for visualization of obstacles in rviz
  Topics:
    - ~input:  [asv_perception_common/ObstacleArray.msg] obstacle messages
    - ~output: [visualization_msgs/MarkerArray] obstacle markers
  Parameters:
    - ~marker_duration_secs:     [uint]    seconds to display a marker.  default: 0
    - ~marker_duration_nsecs:    [uint]    nanoseconds to display a marker.  default:  1e+6*100 (100ms)
    - ~show_position_frame:      [string]  If set, transforms the obstacle pose.position to this frame, and append the transformed position to the obstacle label.  
                                           Currently will only show if obstacle already has a label assigned.  Default:  none
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

      // params
      std::string _show_position_frame;
      int 
        _marker_duration_secs = 0
        , _marker_duration_nsecs = 0
        ;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 