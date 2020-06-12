
#include "ObstacleVisualizationNodelet.h"

#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include "utils.h"

namespace {
    using namespace obstacle_id;

    static const std::string 
        TOPIC_NAME_INPUT = "input"
        , TOPIC_NAME_OUTPUT = "output"
        , MARKER_NS = "obstacle_id"
    ;

    // default for how long should the marker be visible
    static const int MARKER_DURATION_DEFAULT_SECS = 0;
    static const int MARKER_DURATION_DEFAULT_NSECS = 1e+6 * 100;

    // create marker, set common properties
    visualization_msgs::Marker _create_marker( 
        const std_msgs::Header& hdr
        , const asv_perception_common::Obstacle& obs
        , std::int32_t type 
        , const ros::Duration& lifetime
        ) {

        visualization_msgs::Marker marker = {};
        marker.header = hdr;
        
        marker.ns = MARKER_NS + "_" + std::to_string(type);
        marker.type = type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 1.0;
        marker.color.r = marker.color.g = marker.color.b = 0.5;
        
        // if classified type, make it white, else gray
        if ( !obs.label.empty() )
            marker.color.r = marker.color.g = marker.color.b = 1.;

        marker.lifetime = lifetime;

        marker.pose = obs.pose;

        // hack: create marker id from pose
        marker.id = (std::int32_t)( marker.pose.position.x * marker.pose.position.y * marker.pose.position.z ) + type;

        return marker;
    }   // _create_marker

    
    
    visualization_msgs::Marker create_marker_text( 
        const std_msgs::Header& hdr
        , const asv_perception_common::Obstacle& obs
        , const ros::Duration& d
        ) {

        auto marker = _create_marker( hdr, obs, visualization_msgs::Marker::TEXT_VIEW_FACING, d );
        marker.text = !obs.label.empty() ? obs.label : "Unknown";
        marker.scale.z = 2.;// height of uppercase "A"
        marker.pose.position = obs.pose.position;
        marker.pose.position.z += 1.;   // assuming fixed obstacle height

        return marker;
    }
    

    // create a marker for the provided pointcloud, which should be a convex hull
    visualization_msgs::Marker create_marker_cube( const std_msgs::Header& hdr, const asv_perception_common::Obstacle& obs, const ros::Duration& d ) {

        auto marker = _create_marker( hdr, obs, visualization_msgs::Marker::CUBE, d );
        
        const auto minmax = utils::minmax_3d( obs.shape.points );

        marker.scale.x = minmax.second.x - minmax.first.x;
        marker.scale.y = minmax.second.y - minmax.first.y;
        marker.scale.z = minmax.second.z - minmax.first.z;
        
        return marker;
    }   // create_marker_cube

} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleVisualizationNodelet::onInit ()
{
    // Call the super onInit ()
    PCLNodelet::onInit ();

    this->pub_output_ = advertise<visualization_msgs::MarkerArray> ( *pnh_, TOPIC_NAME_OUTPUT, this->max_queue_size_ );

    NODELET_DEBUG("[%s::onInit] Initializing node"
        , getName ().c_str()
    );

    onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleVisualizationNodelet::subscribe ()
{
  this->_sub_input = pnh_->subscribe<asv_perception_common::ObstacleArray> (
    TOPIC_NAME_INPUT
    , this->max_queue_size_
    , bind (&ObstacleVisualizationNodelet::sub_callback, this, _1 )
  );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleVisualizationNodelet::unsubscribe ()
{
  this->_sub_input.shutdown();
}


void ObstacleVisualizationNodelet::sub_callback (
      typename asv_perception_common::ObstacleArray::ConstPtr obs_array
)
{
    // No subscribers/input, no work
    if ( ( this->pub_output_.getNumSubscribers () < 1 ) 
        || !obs_array
        || obs_array->obstacles.empty()
        )
        return;

    // get marker duration
    auto marker_duration_secs = ::MARKER_DURATION_DEFAULT_SECS;
    pnh_->getParamCached( "marker_duration_secs", marker_duration_secs );

    auto marker_duration_nsecs = ::MARKER_DURATION_DEFAULT_NSECS;
    pnh_->getParamCached( "marker_duration_nsecs", marker_duration_nsecs );

    const auto d = ros::Duration( (std::int32_t)marker_duration_secs, (std::int32_t)marker_duration_nsecs );

    // generate markers
    visualization_msgs::MarkerArray marker_array = {};

    for ( const auto& obs : obs_array->obstacles ) {
        marker_array.markers.emplace_back( 
            create_marker_cube( obs_array->header, obs, d ) 
        );

        // text marker
        marker_array.markers.emplace_back( 
            create_marker_text( obs_array->header, obs, d )
        );
    }

    this->pub_output_.publish( marker_array );
}


PLUGINLIB_EXPORT_CLASS(obstacle_id::ObstacleVisualizationNodelet, nodelet::Nodelet)