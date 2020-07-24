#ifndef POINTCLOUDFILTERNODELET_H
#define POINTCLOUDFILTERNODELET_H

#include <pcl_ros/pcl_nodelet.h>
#include <ros/ros.h>

namespace obstacle_id
{
  /*
    Description

    Subscriptions:
        ~input:     [sensor_msgs/PointCloud2] input pointcloud

    Publications:
        ~output:    [sensor_msgs/PointCloud2] filtered pointcloud

    Parameters:
        ~min_distance_[x,y,z]:   [float, default=0]  minimum distance filter, ignored if <= 0

  */
  class PointCloudFilterNodelet
    : public nodelet_topic_tools::NodeletLazy
  {
    public:

      using base_type = nodelet_topic_tools::NodeletLazy;
      
      // default constructor
      PointCloudFilterNodelet() = default;
                                      
    protected:
 
      /** \brief Nodelet initialization routine. */
      void onInit () override;

      /** \brief LazyNodelet connection routine. */
      void subscribe () override;
      void unsubscribe () override;

      // the callback function to handle input from subscription
      void sub_callback ( const sensor_msgs::PointCloud2::ConstPtr& );
      
    private:

        // parameters
        float
            _min_distance_x = 0.f
            , _min_distance_y = 0.f
            , _min_distance_z = 0.f
            ;

        ros::Subscriber sub_;
        ros::Publisher pub_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 