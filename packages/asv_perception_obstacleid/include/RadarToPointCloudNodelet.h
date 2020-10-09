#ifndef RADARTOPOINTCLOUDNODELET_H
#define RADARTOPOINTCLOUDNODELET_H

#include <ros/ros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <asv_perception_common/RadarSegment.h>

namespace obstacle_id
{
  /*
  */
  class RadarToPointCloudNodelet
    : public nodelet_topic_tools::NodeletLazy
  {
    public:

      using base_type = nodelet_topic_tools::NodeletLazy;
      
      // default constructor
      RadarToPointCloudNodelet() = default;
                                      
    protected:
 
      /** \brief Nodelet initialization routine. */
      void onInit () override;

      /** \brief LazyNodelet connection routine. */
      void subscribe () override;
      void unsubscribe () override;

      // the callback function to handle input from subscription
      void sub_callback ( const asv_perception_common::RadarSegment::ConstPtr& );
      
    private:

        ros::Subscriber sub_;
        ros::Publisher pub_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 