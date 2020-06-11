#ifndef OBSTACLE_FUSION_H
#define OBSTACLE_FUSION_H

#include <geometry_msgs/Point32.h>  // geometry_msgs/points is Point32
#include <asv_perception_common/Obstacle.h>

#include "defs.h"
#include "utils.h"
#include "Obstacle2d.h"
#include "Homography.h"

namespace obstacle_id {
namespace obstacle_extraction2d {

namespace impl {

    using namespace asv_perception_common;

    // obstacle real world height, in world units
    static const float OBSTACLE_RW_HEIGHT = 1.f;

    // projects a roi to a polygon using homography
    std::vector<geometry_msgs::Point32> roi_to_polygon( 
        const sensor_msgs::RegionOfInterest& roi
        , const float depth
        , const float height
        , const float z_offset
        , const Homography& h 
        ) 
    {
        const auto make_point = []( float x, float y, float z ) { 
            auto pt = geometry_msgs::Point32();
            pt.x = x;
            pt.y = y;
            pt.z = z;
            return pt;
        };  // make_point

        // construct 2 points at base of detection, other points are built from those
        //  blf = bottom-left-front, brf = bottom-right-front
        const auto 
            blf = h( roi.x_offset, roi.y_offset + roi.height )
            , brf = h( roi.x_offset + roi.width, roi.y_offset + roi.height )
            ;
        
        auto result = std::vector<geometry_msgs::Point32>();
        result.reserve(8);

        // bottom 4 points, counter-clockwise from bottom left front
        result.emplace_back( make_point( blf.first, blf.second, z_offset ) );   // blf
        result.emplace_back( make_point( brf.first, brf.second, z_offset ) );   // brf
        result.emplace_back( make_point( brf.first, brf.second + depth, z_offset ) );   // brr
        result.emplace_back( make_point( blf.first, blf.second + depth, z_offset ) );   // blr

        // top 4 points, clockwise from top left rear
        result.emplace_back( make_point( blf.first, blf.second + depth, z_offset + height ) );   // tlr
        result.emplace_back( make_point( brf.first, brf.second + depth, z_offset + height ) );   // trr
        result.emplace_back( make_point( brf.first, brf.second, z_offset + height ) );   // trf
        result.emplace_back( make_point( blf.first, blf.second, z_offset + height ) );   // tlf

        return result;
    }   // roi_to_polygon    

    Obstacle to_obstacle( const Obstacle2d& obs, const Homography& h ) {

        Obstacle result = {};

        result.label = obs.cls.label;
        result.probability = obs.cls.probability;
        
        // todo:  compute z offset based on parent(s)
        const float z_offset = 0.f;
        
        // real-world height in world units
        //  todo:  estimate real world height based on img shape, location
        const auto height = OBSTACLE_RW_HEIGHT;

        // estimate a real-world depth based on height
        const auto depth = height;
        
        // create points; first point connects to last point
        result.shape.points = roi_to_polygon( obs.cls.roi, depth, height, z_offset, h );

        // todo:  compute centroid for pose.position

        return result;

    }   // to_obstacle
}   // impl

// creates/projects Obstacle messages from a vector of Obstacle2d
inline std::vector<asv_perception_common::Obstacle> project_obstacle2d( 
    const std::vector<std::shared_ptr<Obstacle2d>>& obstacles
    , const Homography& h 
    ) 
{

    // convert obstacles2d to Obstacle msg
    auto result = std::vector<asv_perception_common::Obstacle>();
    for ( auto& obs_ptr : obstacles ) {
        assert( obs_ptr.get() != nullptr );

        result.emplace_back( impl::to_obstacle( *obs_ptr, h ) );
    }

    return result;
}   // create_obstacles

/*
Combines an unclassified obstacle map with a vector of classified obstacles
    Expand classification bounding boxes as needed and create parent/child relationships
    Classified obstacle bounding boxes are then removed from the provided obstacle map
*/
inline std::vector<std::shared_ptr<Obstacle2d>> create_obstacle2d( 
    image_type& obstacle_map
    , const classification2d_vector_type& classifications
)
{
    // check inputs
    if ( classifications.image_height != obstacle_map.rows || classifications.image_width !=  obstacle_map.cols )
        throw std::runtime_error("obstacle map image size does not match classification image size");

    // todo:  expand bounding boxes
    
    //  todo:  estimate z offset via parent/child relationships
    //      depends on adjacent pixel class, known/unknown
    //      assume boat and person are known classes.  
    //      if person located on top of boat, z offset = boat height
    //      if person located on top of unknown class, is person at height of unknown class, 
    //          or does the person's bounding box need expansion?  other solution?

    auto result = std::vector<std::shared_ptr<Obstacle2d>>{};

    // foreach classification:  remove bounding box from map, construct Obstacle2d
    for ( const auto& cls : classifications.classifications ) {
        
        auto obs = std::make_shared<Obstacle2d>(cls);
        obstacle_map( utils::to_cv_rect( cls.roi ) ) = 0;    // set roi to black
        result.emplace_back( std::move( obs ) );
    };  // for


    return result;
}

}}   // ns
#endif