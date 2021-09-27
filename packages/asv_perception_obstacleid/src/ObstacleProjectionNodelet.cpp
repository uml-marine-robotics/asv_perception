// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#include "ObstacleProjectionNodelet.h"

#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include "asv_perception_common/ObstacleArray.h"
#include <cv_bridge/cv_bridge.h>

#include "detail/classified_obstacle_projection.h"
#include "detail/obstacle_projection.h"

namespace {
    using namespace obstacle_id;

    static const std::string 
        TOPIC_NAME_INPUT_SEGMENTATION = "segmentation"
        , TOPIC_NAME_INPUT_IR_SEGMENTATION = "ir_segmentation"
        , TOPIC_NAME_INPUT_CLASSIFICATION = "classification"
        , TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR = "rgb_radar"
        , TOPIC_NAME_INPUT_HOMOGRAPHY_IR_TO_RADAR = "ir_radar"
        , TOPIC_NAME_OUTPUT_OBSTACLES = "obstacles"
        , TOPIC_NAME_OUTPUT_CLOUD = "cloud"
    ;

} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::onInit ()
{
    // Call the super onInit ()
    base_type::onInit ();

    NODELET_DEBUG("[%s::onInit] Initializing node", getName().c_str() );

    // advertise publishers
    this->_pub = advertise<asv_perception_common::ObstacleArray>( *pnh_, TOPIC_NAME_OUTPUT_OBSTACLES, 1 );
    this->_pub_cloud = advertise<sensor_msgs::PointCloud2> (*pnh_, TOPIC_NAME_OUTPUT_CLOUD, 1 );

    // get parameters
    float val = 0;  

    if ( pnh_->getParam("min_height", val ) && ( val > 0. ) )
        this->_min_height = val;

    if ( pnh_->getParam("max_height", val ) && ( val > 0. ) )
        this->_max_height = val;

    if ( pnh_->getParam("min_depth", val ) && ( val > 0. ) )
        this->_min_depth = val;

    if ( pnh_->getParam("max_depth", val ) && ( val > 0. ) )
        this->_max_depth = val;

    if ( pnh_->getParam("resolution", val ) && ( val > 0. ) )
        this->_resolution = val;

    if ( pnh_->getParam("min_distance", val ) && ( val > 0. ) )
        this->_min_distance = val;

    if ( pnh_->getParam("max_distance", val ) && ( val > 0. ) )
        this->_max_distance = val;

    pnh_->getParam("roi_grow_limit", this->_roi_grow_limit );
    pnh_->getParam("roi_shrink_limit", this->_roi_shrink_limit );
    pnh_->getParam("use_segmentation", this->_use_segmentation );
    this->_use_segmentation = false;

    this->_sub_ir_radar = pnh_->subscribe<homography_msg_type>( 
        TOPIC_NAME_INPUT_HOMOGRAPHY_IR_TO_RADAR,
        1,
        bind( &ObstacleProjectionNodelet::cb_homography_ir_radar, this, _1 ));
    
    this->_sub_segmentation_as_classification_only = pnh_->subscribe<classification_msg_type>(TOPIC_NAME_INPUT_IR_SEGMENTATION, 1,
                                                                           bind(&ObstacleProjectionNodelet::sub_ir_seg_callback, this, _1)
                                                                          );

    onInitPostProcess ();

    ROS_DEBUG("ObstacleProjectionNodelet created : %s \n", getName().c_str());
    const std::string& namesp = pnh_->getNamespace();
    ROS_DEBUG("Namespace : %s \n", namesp.c_str());
    const std::string& unresolvedNS = pnh_->getUnresolvedNamespace();
    ROS_DEBUG("Unresolved namespace : %s \n", unresolvedNS.c_str());

    std::string resolvedName_ir2radar = pnh_->resolveName(TOPIC_NAME_INPUT_HOMOGRAPHY_IR_TO_RADAR);
    std::string resolvedName_rgb2radar = pnh_->resolveName(TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR);
    ROS_DEBUG("resolvedName_ir2radar : %s \n", resolvedName_ir2radar.c_str());
    ROS_DEBUG("resolvedName_rgb2radar : %s \n", resolvedName_rgb2radar.c_str());


}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::subscribe ()
{
    ROS_DEBUG("ObstacleProjectionNodelet::subscribe is invoked. \n");
    static const std::uint32_t SYNC_QUEUE_SIZE = 10;

    std::lock_guard<std::mutex> lg( this->_mtx );

    this->_sub_rgb_radar = pnh_->subscribe<homography_msg_type>( 
        TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR
        , 1
        , bind( &ObstacleProjectionNodelet::cb_homography_rgb_radar, this, _1 )
    );

    

    if ( this->_use_segmentation ) { 
    
       ROS_DEBUG("Use segmentation is true. \n");
        //this->_sub_ir_segmentation.subscribe(*pnh_, TOPIC_NAME_INPUT_IR_SEGMENTATION, 1);

        this->_sub_segmentation.subscribe( *pnh_, TOPIC_NAME_INPUT_SEGMENTATION, 1 );
        this->_sub_classification.subscribe( *pnh_, TOPIC_NAME_INPUT_CLASSIFICATION, 1 );
        // There is no IR classification, only segmentation so there is no sync in IR
        // There is only sync in optical because it can have segmentation + classification
        this->_seg_cls_sync.reset( 
            new _seg_cls_synchronizer_type( 
                _seg_cls_sync_policy_type( SYNC_QUEUE_SIZE )
                , _sub_segmentation
                , _sub_classification
            )
        );

        this->_seg_cls_sync->registerCallback( bind (&ObstacleProjectionNodelet::sub_callback, this, _1, _2 ) );    

        
    }
    else {  // classification only

        ROS_DEBUG("Use segmentation is false. \n");
        this->_sub_classification_only = pnh_->subscribe<classification_msg_type> (
            TOPIC_NAME_INPUT_CLASSIFICATION
            , 1
            , bind( &ObstacleProjectionNodelet::sub_callback, this, _1 )
        );

    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::unsubscribe ()
{
    std::lock_guard<std::mutex> lg( this->_mtx );

    if ( this->_use_segmentation ) {
        this->_sub_segmentation.unsubscribe();
        this->_sub_classification.unsubscribe();
        this->_sub_ir_segmentation.unsubscribe();
    } else {
        this->_sub_classification_only.shutdown();
    }

    this->_sub_rgb_radar.shutdown();
    this->_sub_ir_radar.shutdown();
}

void ObstacleProjectionNodelet::cb_homography_ir_radar ( const typename homography_msg_type::ConstPtr& h) {

    if (!h) {
        ROS_DEBUG("NULL IR_RADAR homography matrix received.");
    }
    else {
      ROS_DEBUG("IR_RADAR homography matrix received.");
    }
   this->_h_ir_radar = h;
   
}

void ObstacleProjectionNodelet::cb_homography_rgb_radar( const typename homography_msg_type::ConstPtr& h ) {

    if ( !h ) {
        ROS_DEBUG( "NULL RGB_RADAR homography matrix received." );
        return;
    }
    else {
      ROS_DEBUG("RGB_RADAR homography matrix received.");
    }
    std::lock_guard<std::mutex> lg( this->_mtx );
    this->_h_rgb_radar = h;
}

// only segmentation from IR
void ObstacleProjectionNodelet::sub_ir_seg_callback(const typename classification_msg_type::ConstPtr& seg_msg) {

    // check we have homography, warn
    if ( !this->_h_ir_radar || !seg_msg) {
        ROS_DEBUG( "IR Homography not yet received or seg_msg is null, dropping frame. \n" );
        return;
    }
    else {
        ROS_DEBUG("ObstacleProjectionNodelet::sub_ir_seg_callback callback invoked. \n");
    }
 
  try {

        // construct homography
        const auto h_ir_to_radar = detail::Homography( this->_h_ir_radar->values.data() );

        ROS_DEBUG("h_ir_radar=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", 
              h_ir_to_radar.at(0,0), h_ir_to_radar.at(0,1), h_ir_to_radar.at(0,2), h_ir_to_radar.at(1,0),
              h_ir_to_radar.at(1,1), h_ir_to_radar.at(1,2), h_ir_to_radar.at(2,0), h_ir_to_radar.at(2,1),
              h_ir_to_radar.at(2,2) );
        
        // obstacles projected to frame
        const auto child_frame_id = this->_h_ir_radar->child_frame_id;

        // get classified obstacles, publish obstacle message
        auto msg = asv_perception_common::ObstacleArray();
        auto img = cv::Mat(); // Empty image

        for (const auto& obs2d : (*seg_msg).classifications) {
            ROS_DEBUG("IR obs2d.roi.x_offset=%lf, obs2d.roi.y_offset=%lf, obs2d.roi.width=%lf, obs2d.roi.height=%lf \n",
                (float)obs2d.roi.x_offset,
                (float)obs2d.roi.y_offset,
                (float)obs2d.roi.width,
                (float)obs2d.roi.height);
        }

        msg.obstacles = detail::classified_obstacle_projection::project( 
            img
            , *seg_msg
            , h_ir_to_radar 
            , this->_min_height
            , this->_max_height
            , this->_min_depth
            , this->_max_depth
            , this->_min_distance
            , this->_max_distance
            , this->_roi_grow_limit
            , this->_roi_shrink_limit
            );

        // set headers
        msg.header = seg_msg->header;
        msg.header.frame_id = child_frame_id;

        ROS_DEBUG("child_frame_id=%s for IR homography \n ", child_frame_id.c_str());
        ROS_DEBUG("Number of obstacles in IR after project = %d\n", msg.obstacles.size());

        for ( auto& obs : msg.obstacles ) {
            obs.header = msg.header;
            ROS_DEBUG("IR obs area=%lf, label=%s, label_prob=%lf\n", obs.area, obs.label.c_str(), obs.label_probability);
            ROS_DEBUG("IR obs.position=%lf, %lf, %lf, %lf, %lf, %lf\n", obs.pose.pose.position.x, obs.pose.pose.position.y, obs.pose.pose.position.z,
                                                obs.dimensions.x, obs.dimensions.y, obs.dimensions.z);
        }
            
        this->_pub.publish( msg );
  } catch ( const std::exception& ex ) {
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }

}

// segmentation + classification msg
void ObstacleProjectionNodelet::sub_callback ( 
    const typename segmentation_msg_type::ConstPtr& seg_msg 
    , const typename classification_msg_type::ConstPtr& cls_msg
    )
{

    std::lock_guard<std::mutex> lg( this->_mtx );
    
    // No subscribers/data, no work
    if (
        !seg_msg
        || !cls_msg 
        //|| (
            //( this->_pub.getNumSubscribers () < 1 ) 
            //&& ( this->_pub_cloud.getNumSubscribers() < 1 )
            //)
        )
        return;

    // check we have homography, warn
    if ( !this->_h_rgb_radar ) {
        ROS_DEBUG( "Homography not yet received, dropping frame" );
        return;
    }

    // we have:
    //  seg img, classifications, homography
    auto img_ptr = cv_bridge::toCvCopy( seg_msg, sensor_msgs::image_encodings::MONO8 );
    
    if ( !img_ptr || img_ptr->image.empty() ) {
        NODELET_WARN( "Invalid segmentation image, dropping frame" );
        return;
    }

    // get cv:Mat reference
    auto& img = img_ptr->image;

    if (
        ( img.cols != cls_msg->image_width )
        || ( img.rows != cls_msg->image_height)
    ) {
        NODELET_ERROR( "Segmentation/classification shape mismatch, dropping frame" );
        return;
    }

      ROS_DEBUG("Incoming image size=width=%d, height=%d\n", img.cols, img.rows);

    try {

        // construct homography
        const auto 
            h_rgb_to_radar = detail::Homography( this->_h_rgb_radar->values.data() )
            ;
        // obstacles projected to frame
        const auto child_frame_id = this->_h_rgb_radar->child_frame_id;

        // get classified obstacles, publish obstacle message
        auto msg = asv_perception_common::ObstacleArray();
        
        msg.obstacles = detail::classified_obstacle_projection::project( 
            img
            , *cls_msg
            , h_rgb_to_radar 
            , this->_min_height
            , this->_max_height
            , this->_min_depth
            , this->_max_depth
            , this->_min_distance
            , this->_max_distance
            , this->_roi_grow_limit
            , this->_roi_shrink_limit
            );

        // set headers
        msg.header = cls_msg->header;
        msg.header.frame_id = child_frame_id;

        for ( auto& obs : msg.obstacles ) {
            obs.header = msg.header;
            ROS_DEBUG("seg+cls RGB obs area=%lf, label=%s, label_prob=%lf\n", obs.area, obs.label.c_str(), obs.label_probability);
            ROS_DEBUG("seg+cls RGB obs.position=%lf, %lf, %lf, %lf, %lf, %lf\n", obs.pose.pose.position.x, obs.pose.pose.position.y, obs.pose.pose.position.z,
                                                obs.dimensions.x, obs.dimensions.y, obs.dimensions.z);
        }
            
        this->_pub.publish( msg );

        // debug img
        /*
        if ( this->_pub_debug_img.getNumSubscribers() > 0 ) {
            auto debug_img_msg = sensor_msgs::Image();
            cv_bridge::CvImage debug_img;
            debug_img.encoding = sensor_msgs::image_encodings::MONO8;
            debug_img.image = img;
            debug_img.toImageMsg( debug_img_msg );
            debug_img_msg.header = cls_msg->header;
            this->_pub_debug_img.publish( debug_img_msg );
        }
        */
        
        // unclass pointcloud
        if ( this->_pub_cloud.getNumSubscribers() > 0 ) {

            auto cloud = detail::obstacle_projection::project( 
                img
                , h_rgb_to_radar
                , this->_max_height
                , this->_max_depth
                , this->_resolution
                , this->_max_distance
                );

            sensor_msgs::PointCloud2::Ptr output_blob( new sensor_msgs::PointCloud2() );

            pcl::toROSMsg ( cloud, *output_blob );
            output_blob->header = cls_msg->header;
            output_blob->header.frame_id = child_frame_id;

            // publish
            this->_pub_cloud.publish( output_blob );
        }
  } catch ( const std::exception& ex ) {
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }
    
}

// classification message only
void ObstacleProjectionNodelet::sub_callback ( 
    const typename classification_msg_type::ConstPtr& cls_msg
    )
{

    std::lock_guard<std::mutex> lg( this->_mtx );
    
    // No subscribers/data, no work
    if ( !cls_msg || this->_pub.getNumSubscribers () < 1 )
        return;
        
    // check we have homography, warn
    if ( !this->_h_rgb_radar ) {
        ROS_DEBUG( "Homography not yet received, dropping frame" );
        return;
    } else {
        ROS_DEBUG("RGB homography callback invoked.\n");
    }
    
    try {

        // construct homography
        const auto 
            h_rgb_to_radar = detail::Homography( this->_h_rgb_radar->values.data() )
            ;

        ROS_DEBUG("h_rgb_radar=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", 
              h_rgb_to_radar.at(0,0), h_rgb_to_radar.at(0,1), h_rgb_to_radar.at(0,2), h_rgb_to_radar.at(1,0),
              h_rgb_to_radar.at(1,1), h_rgb_to_radar.at(1,2), h_rgb_to_radar.at(2,0), h_rgb_to_radar.at(2,1),
              h_rgb_to_radar.at(2,2) );
        
        // obstacles projected to frame
        const auto child_frame_id = this->_h_rgb_radar->child_frame_id;

        // get classified obstacles, publish obstacle message
        auto msg = asv_perception_common::ObstacleArray();
        auto img = cv::Mat();

        msg.obstacles = detail::classified_obstacle_projection::project( 
            img
            , *cls_msg
            , h_rgb_to_radar 
            , this->_min_height
            , this->_max_height
            , this->_min_depth
            , this->_max_depth
            , this->_min_distance
            , this->_max_distance
            , this->_roi_grow_limit
            , this->_roi_shrink_limit
            );

        // set headers
        msg.header = cls_msg->header;
        msg.header.frame_id = child_frame_id;


        for ( auto& obs : msg.obstacles ) {
            obs.header = msg.header;
            ROS_DEBUG("RGB obs area=%lf, label=%s, label_prob=%lf\n", obs.area, obs.label.c_str(), obs.label_probability);
            ROS_DEBUG("RGB obs.position=%lf, %lf, %lf, %lf, %lf, %lf\n", obs.pose.pose.position.x, obs.pose.pose.position.y, obs.pose.pose.position.z,
                                                obs.dimensions.x, obs.dimensions.y, obs.dimensions.z);
        }

        ROS_DEBUG("child_frame_id=%s for RGB homography \n ", child_frame_id.c_str());
        ROS_DEBUG("Number of obstacles in RGB after project = %d\n", msg.obstacles.size());
            
        this->_pub.publish( msg );

  } catch ( const std::exception& ex ) {
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }
    
}

PLUGINLIB_EXPORT_CLASS(obstacle_id::ObstacleProjectionNodelet, nodelet::Nodelet)
