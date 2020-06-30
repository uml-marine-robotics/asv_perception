
#include "ObstacleVisualizationNodelet.h"

#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include "defs.h"
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
    static const int MARKER_DURATION_DEFAULT_NSECS = 1e+6 * 100;  // 100ms

    // create marker, set common properties
    visualization_msgs::Marker _create_marker( 
        const asv_perception_common::Obstacle& obs
        , std::int32_t type 
        , const ros::Duration& lifetime
        ) {

        visualization_msgs::Marker marker = {};
        marker.header = obs.header;
        
        marker.ns = MARKER_NS + "_" + std::to_string(type);
        marker.type = type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 1.0;
        marker.color.r = marker.color.g = marker.color.b = 0.5;
        
        // if has label, make it white
        if ( !obs.label.empty() )
            marker.color.r = marker.color.g = marker.color.b = 1.;

        marker.lifetime = lifetime;

        marker.pose = obs.pose;

        // hack: create marker id from pose, type
        marker.id = (std::int32_t)( marker.pose.position.x * marker.pose.position.y * marker.pose.position.z ) + type;

        return marker;
    }   // _create_marker

    
    
    visualization_msgs::Marker create_marker_text( 
        const asv_perception_common::Obstacle& obs
        , const ros::Duration& d
        , const std::string& position_target_frame
        ) {

        auto marker = _create_marker( obs, visualization_msgs::Marker::TEXT_VIEW_FACING, d );

        const bool has_label = !obs.label.empty();

        marker.text = has_label ? obs.label : "Unknown";

        // show position text in target frame?
        if ( has_label && !position_target_frame.empty() ) {
            // get point in target frame
            //  tf may throw
            try {
                const auto position_tf = utils::ros_tf_transform( obs.pose.position, obs.header.frame_id, position_target_frame );
                marker.text += " " + std::to_string(position_tf.x) + " " + std::to_string(position_tf.y) + " " + std::to_string(position_tf.z);
            } catch ( const std::exception& ex ) {
                ROS_ERROR( "[ObstacleVisualizationNodelet] %s", ex.what() );
            }
        }

        marker.scale.z = 2.;// height of uppercase "A"
        marker.pose.position = obs.pose.position;
        marker.pose.position.z += 1.;   // assuming fixed obstacle height

        return marker;
    }
    

    // create a marker for the provided obstacle
    visualization_msgs::Marker create_marker_cube( const asv_perception_common::Obstacle& obs, const ros::Duration& d ) {

        auto marker = _create_marker( obs, visualization_msgs::Marker::CUBE, d );
        
        const auto minmax = utils::minmax_3d( obs.shape.points );

        static const float MIN_SCALE = 0.5;

        marker.scale.x = std::max( minmax.second.x - minmax.first.x, MIN_SCALE );
        marker.scale.y = std::max( minmax.second.y - minmax.first.y, MIN_SCALE );
        marker.scale.z = std::max( minmax.second.z - minmax.first.z, MIN_SCALE );
        
        return marker;
    }   // create_marker_cube

} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleVisualizationNodelet::onInit ()
{
    // Call the super onInit ()
    base_type::onInit ();

    this->_out = advertise<visualization_msgs::MarkerArray> ( *pnh_, TOPIC_NAME_OUTPUT, 1 );

    NODELET_DEBUG("[%s::onInit] Initializing node"
        , getName ().c_str()
    );
    
    pnh_->param( "marker_duration_secs", this->_marker_duration_secs, ::MARKER_DURATION_DEFAULT_SECS );
    pnh_->param( "marker_duration_nsecs", this->_marker_duration_nsecs, ::MARKER_DURATION_DEFAULT_NSECS );
    pnh_->param( "show_position_frame", this->_show_position_frame, std::string() );

    onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleVisualizationNodelet::subscribe ()
{
  this->_in = pnh_->subscribe<asv_perception_common::ObstacleArray> (
    TOPIC_NAME_INPUT
    , 1
    , bind (&ObstacleVisualizationNodelet::sub_callback, this, _1 )
  );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleVisualizationNodelet::unsubscribe ()
{
  this->_in.shutdown();
}


void ObstacleVisualizationNodelet::sub_callback (
      const typename asv_perception_common::ObstacleArray::ConstPtr& obs_array
)
{
    // No subscribers/input, no work
    if ( ( this->_out.getNumSubscribers () < 1 ) 
        || !obs_array
        || obs_array->obstacles.empty()
        )
        return;

    const auto d = ros::Duration( (std::int32_t)this->_marker_duration_secs, (std::int32_t)this->_marker_duration_nsecs );

    // generate markers
    visualization_msgs::MarkerArray marker_array = {};

    for ( const auto& obs : obs_array->obstacles ) {
        marker_array.markers.emplace_back( 
            create_marker_cube( obs, d ) 
        );

        // text marker if not unknown
        if ( !obs.label.empty() )
            marker_array.markers.emplace_back( 
                create_marker_text( obs, d, this->_show_position_frame )
            );
    }

    this->_out.publish( marker_array );
}


PLUGINLIB_EXPORT_CLASS(obstacle_id::ObstacleVisualizationNodelet, nodelet::Nodelet)