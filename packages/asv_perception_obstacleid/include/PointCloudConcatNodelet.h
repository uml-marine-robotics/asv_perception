#ifndef POINTCLOUDCONCATNODELET_H
#define POINTCLOUDCONCATNODELET_H

#include <mutex>
#include <pcl_ros/pcl_nodelet.h>
#include <ros/ros.h>

namespace obstacle_id
{
  /*
    Concatenates multiple partial pointcloud segments received over time into a single pointcloud

    Subscriptions:
        ~input:     [sensor_msgs/PointCloud2] input pointcloud segment

    Publications:
        ~full:      [sensor_msgs/PointCloud2]  pointcloud representing 1 complete pointcloud after having received all N segments
        ~current:   [sensor_msgs/PointCloud2]  pointcloud representing most current information, published after receiving each segment

    Parameters:
        ~segments:   [int, required]  number of segments to accumulate before publishing

  */
  class PointCloudConcatNodelet
    : public nodelet_topic_tools::NodeletLazy
  {
    public:

      using base_type = nodelet_topic_tools::NodeletLazy;
      
      // default constructor
      PointCloudConcatNodelet() = default;
                                      
    protected:
 
      /** \brief Nodelet initialization routine. */
      void onInit () override;

      /** \brief LazyNodelet connection routine. */
      void subscribe () override;
      void unsubscribe () override;

      // the callback function to handle input from subscription
      void sub_callback ( const sensor_msgs::PointCloud2::ConstPtr& );
      
    private:

        ros::Subscriber sub_;
        ros::Publisher 
          pub_full_
          , pub_current_
          ;

        std::mutex mtx_;
        using lock_type_ = std::lock_guard<std::mutex>;

        // num segments
        int nsegments_ = 0;
        // segments storage      
        std::vector<sensor_msgs::PointCloud2::ConstPtr> segments_;
        // current index in storage
        int current_idx_ = 0;

        

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 