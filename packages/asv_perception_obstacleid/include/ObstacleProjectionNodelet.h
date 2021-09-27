// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#ifndef OBSTACLEPROJECTIONNODELET_H
#define OBSTACLEPROJECTIONNODELET_H

#include <mutex>
#include <ros/ros.h>
#include <nodelet_topic_tools/nodelet_lazy.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <asv_perception_common/Homography.h>
#include <asv_perception_common/ClassificationArray.h>

#include "defs.h"

namespace obstacle_id
{
    /*
    Nodelet for obstacle backprojection from 2D to 3D.  
    In 2D, combines segmented obstacle map with classified obstacle bounding boxes
    Expands classified obstacle bounding boxes as needed, estimates 3d properties, then creates Obstacle messages for the classified obstacles
    Projects remaining unclassified obstacle pixels to pointcloud
    Subscriptions:
        ~segmentation:      [sensor_msgs/Image] 2d obstacle map, all pixels with value > 0 are considered obstacle pixels
        *~segmentation:     [asv_perception_common/ClassificationArray]  Classifications : From IR, segmentation is
                             modified to output ClassificationArray
        ~classification:    [asv_perception_common/ClassificationArray]  Classifications
        ~rgb_radar:         [asv_perception_common/Homography] rgb to radar homography matrix
    
    Publications:
        ~obstacles:         [asv_perception_common/ObstacleArray] Classified obstacles
        ~cloud:             [sensor_msgs/PointCloud2]  Unclassified obstacle pointcloud, if segmentation is enabled

    Parameters:
        ~use_segmentation   [bool, default=true]  Use segmentation image
        ~min_height         [float, default=1.0]  Minimum projected obstacle height
        ~max_height         [float, default=1.0]  Maximum projected obstacle height
        ~min_depth          [float, default=1.0]  Minimum projected obstacle depth
        ~max_depth          [float, default=1.0]  Maximum projected obstacle depth
        ~resolution         [float, default=0.25]  Obstacle pointcloud resolution (space between points)
        ~min_distance       [float, default=3.0]  Minimum projected obstacle distance
        ~max_distance       [float, default=100.0]  Maximum projected obstacle distance
        ~roi_shrink_limit   [float, default=0]    Percentage limit of how much a classified obstacle ROI can shrink
        ~roi_grow_limit     [float, default=0]    Percentage limit of how much a classified obstacle ROI can grow
    */
    class ObstacleProjectionNodelet 
    : public nodelet_topic_tools::NodeletLazy
    {
    public:

        using base_type = nodelet_topic_tools::NodeletLazy;
        using segmentation_msg_type = sensor_msgs::Image;
        using classification_msg_type = asv_perception_common::ClassificationArray;
        using homography_msg_type = asv_perception_common::Homography;
        
        // default constructor
        ObstacleProjectionNodelet() = default;

    protected:

        /** \brief Nodelet initialization routine. */
        void onInit () override;

        /** \brief LazyNodelet connection routine. */
        void subscribe () override;
        void unsubscribe () override;

        // the callback function to handle input ( classification + segmentation msgs )
        void sub_callback ( 
            const typename segmentation_msg_type::ConstPtr&
            , const typename classification_msg_type::ConstPtr&
        );

        // the callback function to handle input ( classification msg only )
        void sub_callback ( 
            const typename classification_msg_type::ConstPtr&
        );

        void sub_ir_seg_callback(const typename classification_msg_type::ConstPtr& );

        // homography callback
        void cb_homography_rgb_radar ( const typename homography_msg_type::ConstPtr& );

        void cb_homography_ir_radar ( const typename homography_msg_type::ConstPtr& h);
        
    private:

        // state protection mutex
        std::mutex _mtx;

        // publishers
        ros::Publisher _pub, _pub_cloud;

        // subscriptions
        ros::Subscriber _sub_rgb_radar;
            // only used if !use_segmentation
        ros::Subscriber _sub_classification_only;
        ros::Subscriber _sub_ir_radar;
        ros::Subscriber _sub_segmentation_as_classification_only;

        message_filters::Subscriber<segmentation_msg_type> _sub_segmentation;
        message_filters::Subscriber<classification_msg_type> _sub_ir_segmentation;
        message_filters::Subscriber<classification_msg_type> _sub_classification;

        // sync policy for segmentation + classification subscriptions
        using _seg_cls_sync_policy_type = message_filters::sync_policies::ApproximateTime<segmentation_msg_type, classification_msg_type>;
        using _seg_cls_synchronizer_type = message_filters::Synchronizer<_seg_cls_sync_policy_type>;
        boost::shared_ptr<_seg_cls_synchronizer_type> _seg_cls_sync;

        // homography msg storage
        typename asv_perception_common::Homography::ConstPtr _h_rgb_radar;
        typename asv_perception_common::Homography::ConstPtr _h_ir_radar;

        // parameters
        bool _use_segmentation = true;

        float 
            _min_height = 1.f
            , _max_height = 1.f
            , _min_depth = 1.f
            , _max_depth = 1.f
            , _resolution = 0.25f
            , _min_distance = 3.f
            , _max_distance = 100.f
            , _roi_grow_limit = 0.f
            , _roi_shrink_limit = 0.f
            ;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};  // class
}   // ns

#endif  //#ifndef