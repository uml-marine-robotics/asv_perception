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
#include <asv_perception_common/Homography.h>

namespace {
    using namespace obstacle_id;

    static const std::string 
        TOPIC_NAME_INPUT_SEGMENTATION = "segmentation"
        , TOPIC_NAME_INPUT_CLASSIFICATION = "classification"
        , TOPIC_NAME_INPUT_HOMOGRAPHY = "homography"
        , TOPIC_NAME_OUTPUT = "output"
        , PARAM_NAME_PIXELMAP = "pixel_map"
    ;

} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::onInit ()
{
    // Call the super onInit ()
    PCLNodelet::onInit ();

    // output node
    // this->pub_output_ = advertise<visualization_msgs::MarkerArray> ( *pnh_, TOPIC_NAME_OUTPUT, this->max_queue_size_ );

    NODELET_DEBUG("[%s::onInit] Initializing node"
        , getName ().c_str()
    );

    onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::subscribe ()
{

    static const std::uint32_t SYNC_QUEUE_SIZE = 10;

    this->_sub_segmentation.subscribe( *pnh_, TOPIC_NAME_INPUT_SEGMENTATION, 1 );

    this->_seg_cls_sync.reset( new _seg_cls_synchronizer_type( _seg_cls_sync_policy_type( SYNC_QUEUE_SIZE ), _sub_segmentation, _sub_classification  ));

    this->_seg_cls_sync->registerCallback( bind (&ObstacleProjectionNodelet::sub_callback, this, _1, _2 ) );

    /*
    this->_sub_input = pnh_->subscribe<sensor_msgs::PointCloud2> (
        TOPIC_NAME_INPUT
        , this->max_queue_size_
        , bind (&ObstacleProjectionNodelet::sub_callback, this, _1 )
    );
    */
  

}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::unsubscribe ()
{
  //this->_sub_input.shutdown();
}


void ObstacleProjectionNodelet::sub_callback ( typename segmentation_msg_type::ConstPtr, typename classification_msg_type::ConstPtr )
{
    // No subscribers, no work
    if ( this->pub_output_.getNumSubscribers () <= 0 )
        return;

    // we have:
    //  seg img, classifications, homography, pixelmap

}


PLUGINLIB_EXPORT_CLASS(obstacle_id::ObstacleProjectionNodelet, nodelet::Nodelet)
