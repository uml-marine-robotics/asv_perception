// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

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
    ~cluster_tolerance: [float]               clustering tolerance
    ~cluster_size_min:  [uint, default=1]     minimum number of points in a cluster
    ~cluster_size_max:  [uint, default=max]   maximum number of points in a cluster
    ~cluster_area_min:  [float, default=0]    minimum convex hull area
    ~cluster_area_max:  [float, default=max]  maximum convex hull area
  
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
          cluster_sz_min_ = 1
          , cluster_sz_max_ = std::numeric_limits<std::uint32_t>::max()
        ;

        float
            cluster_tolerance_ = 0.f
            , cluster_area_min_ = 0.f
            , cluster_area_max_ = std::numeric_limits<float>::max()
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