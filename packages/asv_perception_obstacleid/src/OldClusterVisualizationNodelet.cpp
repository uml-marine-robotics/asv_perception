
#include "ObstacleVisualizationNodelet.h"

#include <pluginlib/class_list_macros.h>
#include <pcl/common/centroid.h>    // compute3DCentroid
#include <pcl/common/common.h>      // getMinMax3D
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/MarkerArray.h>

namespace {
    using namespace obstacle_id;

    static const std::string 
        TOPIC_NAME_INPUT = "input"
        , TOPIC_NAME_OUTPUT = "output"
        , MARKER_NS = "obstacle_id"
    ;

    // how long should the marker be visible.  should probably be ros parameter
    static const std::uint32_t MARKER_DURATION_SECONDS = 3;

    std::string labelToString( std::uint32_t label ) {
        // todo:  need a dictionary or something for label --> string
        std::string result = "Unknown";
        switch ( label ) {
            case 255:
                result = "Boat";
                break;
        };

        return result;
    }       // labelToString


    // create marker, set common properties
    visualization_msgs::Marker _create_marker( const pointcloud_type& pc, std::int32_t type ) {

        visualization_msgs::Marker marker = {};
        pcl_conversions::fromPCL( pc.header, marker.header);
        marker.ns = MARKER_NS + "_" + std::to_string(type);
        marker.type = type;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration( MARKER_DURATION_SECONDS, 0 );

        // compute centroid, generate unique id for this marker
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid( pc, centroid );
        marker.id = (std::int32_t)( centroid[0]*centroid[1] ) ; // hack

        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];

        const auto& first_pt = pc.points.front();
        const auto label = first_pt.label;
        marker.color.r = label / 255.;
        marker.color.g = label / 255.;
        marker.color.b = label / 255.;

        return marker;
    }   // _create_marker

    // create a marker for the provided pointcloud, which should be a convex hull
    visualization_msgs::Marker create_marker_linestrip( const pointcloud_type& pc ) {

        auto marker = _create_marker(pc, visualization_msgs::Marker::LINE_STRIP );
        
        marker.scale.x = 0.25;
        marker.scale.y = 0;
        marker.scale.z = 0;
        
        for ( const auto& pt : pc.points ) {

            // marker.points is vector of ::geometry_msgs::Point            
            auto marker_pt = geometry_msgs::Point();
            marker_pt.x = pt.x;
            marker_pt.y = pt.y;
            marker_pt.z = pt.z;
            marker.points.emplace_back( std::move( marker_pt ) );
        }

        //
        marker.points.emplace_back( marker.points.front() );

        return marker;
    }   // create_marker_linestrip

    visualization_msgs::Marker create_marker_text( const pointcloud_type& pc ) {

        auto marker = _create_marker(pc, visualization_msgs::Marker::TEXT_VIEW_FACING );

        const auto& first_pt = pc.points.front();
        marker.text = labelToString( first_pt.label );
        marker.scale.z = 2.;// height of uppercase "A"

        point_type min_pt, max_pt;
        pcl::getMinMax3D(pc, min_pt, max_pt);

        marker.pose.position.z = max_pt.z + 1.0;  // move the text above the max point (is this max z?)

        return marker;
    }

    // create a marker for the provided pointcloud, which should be a convex hull
    visualization_msgs::Marker create_marker_cube( const pointcloud_type& pc ) {

        auto marker = _create_marker(pc, visualization_msgs::Marker::CUBE );
        
        point_type min_pt, max_pt;
        pcl::getMinMax3D(pc, min_pt, max_pt);

        marker.scale.x = max_pt.x - min_pt.x;
        marker.scale.y = max_pt.y - min_pt.y;
        marker.scale.z = max_pt.z - min_pt.z;
        
        return marker;
    }   // create_marker_linestrip

    

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
  this->_sub_input = pnh_->subscribe<sensor_msgs::PointCloud2> (
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
      const sensor_msgs::PointCloud2::ConstPtr& cloud
)
{
    // No subscribers, no work
    if ( this->pub_output_.getNumSubscribers () <= 0 )
        return;

    // If cloud is given, check if it's valid
    if (!isValid (cloud))
    {
        NODELET_ERROR ("[%s::sub_callback] Invalid input!", getName().c_str() );
        return;
    }

    pointcloud_type pc = {};
    pcl::fromROSMsg( *cloud, pc );

    // empty cloud, no work
    if ( pc.empty() )
        return;

    // generate markers
    visualization_msgs::MarkerArray marker_array = {};
    marker_array.markers.emplace_back( create_marker_cube( pc ) );
    marker_array.markers.emplace_back( create_marker_text( pc ) );

    this->pub_output_.publish( marker_array );
}


PLUGINLIB_EXPORT_CLASS(obstacle_id::ObstacleVisualizationNodelet, nodelet::Nodelet)