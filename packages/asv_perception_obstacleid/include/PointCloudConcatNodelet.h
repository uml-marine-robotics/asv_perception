// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

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
        ~decay_time:   [int, required]  number of seconds to store a partial pointcloud.  A full pointcloud is published after each decay_time interval

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

        // segments, time storage      
        std::vector<std::pair<sensor_msgs::PointCloud2::ConstPtr, ros::Time>> segments_;

        // segment survival time
        float decay_time_ = 0.f;

        // last full publish time
        ros::Time last_full_publish_ = ros::Time::now();

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 