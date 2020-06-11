#ifndef OBSTACLE_FUSION_H
#define OBSTACLE_FUSION_H

#include <asv_perception_common/Obstacle.h>

#include "defs.h"
#include "utils.h"
#include "Obstacle2d.h"
#include "Homography.h"


namespace obstacle_id {
namespace obstacle_extraction2d {

namespace impl {

    using namespace asv_perception_common;

    Obstacle to_obstacle( const Obstacle2d& obs, const Homography& h ) {

        Obstacle result = {};

        result.label = obs.cls.label;
        result.probability = obs.cls.probability;
        
        // todo:  compute z offset based on parent(s)

        // compute polygon
        auto foo = h( 0, 0 );



        // compute centroid for pose.position
        
        
        return result;

    }   // to_obstacle
}   // impl

/*
Combines an unclassified obstacle map with a vector of classified obstacles
    Classified obstacle bounding boxes are then removed from the provided obstacle map
    Returns vector of Obstacle messages
*/
inline std::vector<asv_perception_common::Obstacle> extract( 
    image_type& obstacle_map
    , const classification2d_vector_type& classifications
    , const Homography& h
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

    auto obstacles = std::vector<std::shared_ptr<Obstacle2d>>{};

    // foreach classification:  remove bounding box from map, construct Obstacle2d
    for ( const auto& cls : classifications.classifications ) {
        
        auto obs = std::make_shared<Obstacle2d>(cls);
        obstacle_map( utils::to_cv_rect( cls.roi ) ) = 0;    // set roi to black
        obstacles.emplace_back( std::move( obs ) );
    };  // for


    // convert obstacles2d to Obstacle msg
    auto result = std::vector<asv_perception_common::Obstacle>();
    for ( auto& obs : obstacles ) {
        assert( obs.get() != nullptr );
        result.emplace_back( impl::to_obstacle( *obs, h ) );
    }

    return result;
}

}}   // ns
#endif