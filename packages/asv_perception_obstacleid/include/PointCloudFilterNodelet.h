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
        ~min_distance_[x,y,z]:   [float, default=0]  minimum distance filter (from origin) for dimension x/y/z, ignored if <= 0
        ~min_distance:           [float, default=0]  minimum distance filter (radius from origin), ignored if <= 0
        ~outlier_min_neighbors:  [int, default=0]    minimum number of neighbors within `outlier_radius` for a point to be excluded from removal.  0 means ignored
        ~outlier_radius:         [float, default=0]  radius for outlier removal
        
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
            min_distance_x_ = 0.f
            , min_distance_y_ = 0.f
            , min_distance_z_ = 0.f
            , min_distance_ = 0.f
            , outlier_radius_ = 0.f
            ;
        int 
          outlier_min_neighbors_ = 0
          ;

        ros::Subscriber sub_;
        ros::Publisher pub_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 