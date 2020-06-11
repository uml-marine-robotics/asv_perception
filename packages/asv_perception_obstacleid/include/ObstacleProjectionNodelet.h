#ifndef ASV_PERCEPTION_OBSTACLEPROJECTIONNODELET_H
#define ASV_PERCEPTION_OBSTACLEPROJECTIONNODELET_H

#include <ros/ros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <asv_perception_common/Homography.h>

#include "defs.h"

namespace obstacle_id
{
    /*
    Nodelet for obstacle backprojection from 2D to 3D.  
    In 2D, combines segmented obstacles with obstacle bounding boxes, then backprojects a labeled pointcloud
    Topics:
    - segmentation:       2d obstacle map of unknown obstacle types
    - classification:     ClassificationArray
    - homography:         2d to 3d homography matrix
    - pixel_map:          dictionary of classification: pixel_value (string, uint8)
    - output:             labeled pointcloud
    */
    class ObstacleProjectionNodelet 
    : public pcl_ros::PCLNodelet
    {
    public:

        using segmentation_msg_type = sensor_msgs::Image;
        using classification_msg_type = classification2d_vector_type;
        using homography_msg_type = asv_perception_common::Homography;
        //using pixelmap_param_type = classification_pixel_map_type;
        
        // default constructor
        ObstacleProjectionNodelet() = default;
                                        
    protected:

        /** \brief Nodelet initialization routine. */
        void onInit () override;

        /** \brief LazyNodelet connection routine. */
        void subscribe () override;
        void unsubscribe () override;

        // the callback function to handle input from subscription
        void sub_callback ( typename segmentation_msg_type::ConstPtr, typename classification_msg_type::ConstPtr );
        
    private:

        // subscriptions
        ros::Subscriber _sub_homography;
        message_filters::Subscriber<segmentation_msg_type> _sub_segmentation;
        message_filters::Subscriber<classification_msg_type> _sub_classification;

        // sync policy for segmentation + classification subscriptions
        using _seg_cls_sync_policy_type = message_filters::sync_policies::ApproximateTime<segmentation_msg_type, classification_msg_type>;
        using _seg_cls_synchronizer_type = message_filters::Synchronizer<_seg_cls_sync_policy_type>;
        boost::shared_ptr<_seg_cls_synchronizer_type> _seg_cls_sync;

        // pixel map
        // pixelmap_param_type _pixelmap;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};  // class
}

#endif  //#ifndef