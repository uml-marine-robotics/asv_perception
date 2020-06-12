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

#include "ClassifiedObstacleProjectionNodelet.h"

#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <asv_perception_common/ObstacleArray.h>
#include <cv_bridge/cv_bridge.h>

#include "classified_obstacle_projection.h"

namespace {
    using namespace obstacle_id;

    static const std::string 
        TOPIC_NAME_INPUT_SEGMENTATION = "segmentation"
        , TOPIC_NAME_INPUT_CLASSIFICATION = "classification"
        , TOPIC_NAME_INPUT_HOMOGRAPHY = "homography"
        , TOPIC_NAME_OUTPUT_OBSTACLES = "output"
        , TOPIC_NAME_OUTPUT_SEGMENTATION = "output_segmentation"
    ;

} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void ClassifiedObstacleProjectionNodelet::onInit ()
{
    // Call the super onInit ()
    base_type::onInit ();

    // advertise publishers
    this->_pub_obstacles = advertise<asv_perception_common::ObstacleArray>( *pnh_, TOPIC_NAME_OUTPUT_OBSTACLES, 1 );
    this->_pub_segmentation = advertise<segmentation_msg_type>( *pnh_, TOPIC_NAME_OUTPUT_SEGMENTATION, 1 );

    NODELET_DEBUG("[%s::onInit] Initializing node"
        , getName ().c_str()
    );

    onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ClassifiedObstacleProjectionNodelet::subscribe ()
{
    static const std::uint32_t SYNC_QUEUE_SIZE = 10;

    this->_sub_homography = pnh_->subscribe<homography_msg_type>( 
        TOPIC_NAME_INPUT_HOMOGRAPHY
        , 1
        , bind( &ClassifiedObstacleProjectionNodelet::cb_homography, this, _1 )
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

    this->_seg_cls_sync->registerCallback( bind (&ClassifiedObstacleProjectionNodelet::sub_callback, this, _1, _2 ) );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ClassifiedObstacleProjectionNodelet::unsubscribe ()
{
    this->_sub_segmentation.unsubscribe();
    this->_sub_classification.unsubscribe();
    this->_sub_homography.shutdown();
}

void ClassifiedObstacleProjectionNodelet::cb_homography ( typename homography_msg_type::ConstPtr h ) {

    if ( !h ) {
        NODELET_WARN( "Invalid homography received, ignoring" );
        return;
    }

    this->_homography = h;
}

void ClassifiedObstacleProjectionNodelet::sub_callback ( 
    typename segmentation_msg_type::ConstPtr seg_msg 
    , typename classification_msg_type::ConstPtr cls_msg
    )
{
    // No subscribers/data, no work
    if (
        !seg_msg
        || !cls_msg 
        || (
            ( this->_pub_obstacles.getNumSubscribers () < 1 ) 
            && ( this->_pub_segmentation.getNumSubscribers() < 1 ) 
            )
        )
        return;

    // check we have homography, warn
    if ( !this->_homography ) {
        NODELET_WARN( "Homography not yet received, dropping frame" );
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

    // publish obstacle message
    auto obs_msg = asv_perception_common::ObstacleArray();
    obs_msg.header = cls_msg->header;
    obs_msg.header.frame_id = "map";    // parameterize?
    obs_msg.obstacles = classified_obstacle_projection::create_obstacles( 
        img
        , *cls_msg
        , Homography( this->_homography->values.data() )
        );
    this->_pub_obstacles.publish( obs_msg );

    // publish filtered segmentation message
    auto seg_msg_out = img_ptr->toImageMsg();
    seg_msg_out->header = seg_msg->header;
    this->_pub_segmentation.publish( seg_msg_out ); // need a shared_ptr for nodelet optimization?
}


PLUGINLIB_EXPORT_CLASS(obstacle_id::ClassifiedObstacleProjectionNodelet, nodelet::Nodelet)
