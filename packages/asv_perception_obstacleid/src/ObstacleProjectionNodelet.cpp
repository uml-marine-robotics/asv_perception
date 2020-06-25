// class to take gs image
//  (?) obstacle boundary expansion into unknown obstacle area on margins
//  extract known obstacle types by class
//      establish parent-child relationships so location is properly set (where is the man on the boat?)
//  backproject to PC
// unknown obstacles...logic is no different?  see rgb_to_pointcloud
//  if no different, no need to expand/etc or set parent-child relationships

// class takes wasr img and classifier bbs
//  expands class bottom border (refines both bbs and wasr img)
//  organizes classes into heirarchy as needed
//  projects to pc.  known obstacles as a unit.  unknown via scanline method

#include "ObstacleProjectionNodelet.h"

#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <asv_perception_common/ObstacleArray.h>
#include <cv_bridge/cv_bridge.h>

#include "detail/classified_obstacle_projection.h"
#include "detail/obstacle_projection.h"

namespace {
    using namespace obstacle_id;

    static const std::string 
        TOPIC_NAME_INPUT_SEGMENTATION = "segmentation"
        , TOPIC_NAME_INPUT_CLASSIFICATION = "classification"
        , TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR = "rgb_radar"
        , TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADARIMG = "rgb_radarimg"
        , TOPIC_NAME_OUTPUT_OBSTACLES = "obstacles"
        , TOPIC_NAME_OUTPUT_CLOUD = "cloud"
    ;

} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::onInit ()
{
    // Call the super onInit ()
    base_type::onInit ();

    // advertise publishers
    this->_pub = advertise<asv_perception_common::ObstacleArray>( *pnh_, TOPIC_NAME_OUTPUT_OBSTACLES, 1 );
    this->_pub_cloud = advertise<sensor_msgs::PointCloud2> (*pnh_, TOPIC_NAME_OUTPUT_CLOUD, 1 );

    NODELET_DEBUG("[%s::onInit] Initializing node", getName().c_str() );

    onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::subscribe ()
{
    static const std::uint32_t SYNC_QUEUE_SIZE = 10;

    this->_sub_rgb_radarimg = pnh_->subscribe<homography_msg_type>( 
        TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADARIMG
        , 1
        , bind( &ObstacleProjectionNodelet::cb_homography_rgb_radarimg, this, _1 )
    );

    this->_sub_rgb_radar = pnh_->subscribe<homography_msg_type>( 
        TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR
        , 1
        , bind( &ObstacleProjectionNodelet::cb_homography_rgb_radar, this, _1 )
    );
    
    this->_sub_segmentation.subscribe( *pnh_, TOPIC_NAME_INPUT_SEGMENTATION, 1 );
    this->_sub_classification.subscribe( *pnh_, TOPIC_NAME_INPUT_CLASSIFICATION, 1 );

    this->_seg_cls_sync.reset( 
        new _seg_cls_synchronizer_type( 
            _seg_cls_sync_policy_type( SYNC_QUEUE_SIZE )
            , _sub_segmentation
            , _sub_classification
        )
    );

    this->_seg_cls_sync->registerCallback( bind (&ObstacleProjectionNodelet::sub_callback, this, _1, _2 ) );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::unsubscribe ()
{
    this->_sub_segmentation.unsubscribe();
    this->_sub_classification.unsubscribe();
    this->_sub_rgb_radarimg.shutdown();
    this->_sub_rgb_radar.shutdown();
}

void ObstacleProjectionNodelet::cb_homography_rgb_radarimg( const typename homography_msg_type::ConstPtr& h ) {

    if ( !h ) {
        NODELET_WARN( "Invalid homography received, ignoring" );
        return;
    }
    this->_h_rgb_radarimg = h;
}

void ObstacleProjectionNodelet::cb_homography_rgb_radar( const typename homography_msg_type::ConstPtr& h ) {

    if ( !h ) {
        NODELET_WARN( "Invalid homography received, ignoring" );
        return;
    }
    this->_h_rgb_radar = h;
}

void ObstacleProjectionNodelet::sub_callback ( 
    const typename segmentation_msg_type::ConstPtr& seg_msg 
    , const typename classification_msg_type::ConstPtr& cls_msg
    )
{

    // No subscribers/data, no work
    if (
        !seg_msg
        || !cls_msg 
        || (
            ( this->_pub.getNumSubscribers () < 1 ) 
            && ( this->_pub_cloud.getNumSubscribers() < 1 )
            )
        )
        return;

    // check we have homography, warn
    if ( !this->_h_rgb_radarimg || !this->_h_rgb_radar ) {
        NODELET_WARN( "Homographies not yet received, dropping frame" );
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
        NODELET_WARN( "Segmentation/classification shape mismatch, dropping frame" );
        return;
    }

    try {

        // construct homography
        const auto 
            h_rgb_to_radar = detail::Homography( this->_h_rgb_radar->values.data() )
            , h_radar_to_rgbimg = detail::Homography( this->_h_rgb_radarimg->values.data() ).inverse()
            ;
        // obstacles projected to frame
        const auto& child_frame_id = this->_h_rgb_radar->child_frame_id;

        // get classified obstacles, publish obstacle message
        auto msg = asv_perception_common::ObstacleArray();
        msg.header = cls_msg->header;
        msg.header.frame_id = child_frame_id;
        msg.obstacles = detail::classified_obstacle_projection::project( img, *cls_msg, h_rgb_to_radar );
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

            auto cloud = detail::obstacle_projection::project( img, h_rgb_to_radar, h_radar_to_rgbimg );

            sensor_msgs::PointCloud2::Ptr output_blob( new sensor_msgs::PointCloud2() );
            
            pcl::toROSMsg ( cloud, *output_blob );
            output_blob->header = cls_msg->header;
            output_blob->header.frame_id = child_frame_id;

            // publish
            this->_pub_cloud.publish( output_blob );
        }
    }
    catch ( const std::exception& ex ) {
        NODELET_ERROR( "[ObstacleProjectionNodelet] %s", ex.what() );
    }
    
}

PLUGINLIB_EXPORT_CLASS(obstacle_id::ObstacleProjectionNodelet, nodelet::Nodelet)
