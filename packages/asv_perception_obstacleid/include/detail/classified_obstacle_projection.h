#ifndef CLASSIFIED_OBSTACLE_PROJECTION_H
#define CLASSIFIED_OBSTACLE_PROJECTION_H

#include <geometry_msgs/Point32.h>  // geometry_msgs/Polygon.points is Point32
#include <asv_perception_common/Obstacle.h>
#include <asv_perception_common/ClassificationArray.h>

#include "defs.h"
#include "utils.h"
#include "ClassifiedObstacle2d.h"
#include "Homography.h"

namespace obstacle_id {
namespace detail {
namespace classified_obstacle_projection {

using namespace asv_perception_common;

/*
Combines an unclassified obstacle map with a vector of classification bounding boxes
    Expand classification bounding boxes as needed and create parent/child relationships
    Classified obstacle bounding boxes are then removed from the provided obstacle map
    Returns vector of projected Obstacles
*/
inline std::vector<Obstacle> project( 
    image_type& obstacle_map
    , const asv_perception_common::ClassificationArray& classifications
    , const Homography& h
    , const float min_height
    , const float max_height
    , const float min_depth
    , const float max_depth
    , const float max_distance
)
{
    // check inputs
    if ( classifications.image_height != obstacle_map.rows || classifications.image_width !=  obstacle_map.cols )
        throw std::runtime_error("obstacle map image size does not match classification image size");

    // foreach classification, construct Obstacle2d
    auto obstacles_2d = std::vector<std::shared_ptr<ClassifiedObstacle2d>>();
    for ( const auto& cls : classifications.classifications ) 
        obstacles_2d.emplace_back( std::make_shared<ClassifiedObstacle2d>(cls) );

    //  todo:  expand/contract bounding boxes
    //  todo:  estimate z offset via parent/child relationships
    //      depends on adjacent pixel class, known/unknown
    //      eg assume boat and person are known classes.  
    //      if person located on top of boat, z offset = boat height?
    //      if person located on top of unknown class, is person at height of unknown class, 
    //          or does the person's bounding box need expansion?  other solution?

    // now project each to obstacle msg, remove bb from unknown obstacle map
    auto result = std::vector<Obstacle>{};
    const auto max_distance_squared = std::pow( max_distance, 2.f );

    for ( const auto& obs2d : obstacles_2d ) {

        auto obs = obs2d->project( h, min_height, max_height, min_depth, max_depth );

        // max distance check, if supplied
        if ( max_distance > 0.f ) {
            const auto dist_squared = 
                std::pow( obs.pose.pose.position.x, 2.f ) 
                + std::pow(obs.pose.pose.position.y, 2.f )
                + std::pow(obs.pose.pose.position.z, 2.f )
            ;

            if ( dist_squared > max_distance_squared )
                continue;
        }

        const auto rect = utils::to_cv_rect( obs2d->cls.roi, obstacle_map );    // get opencv rect based on roi
        obstacle_map( rect ) = 0;    // set roi to black in obstacle_map
        result.emplace_back( std::move(obs) );
    }

    return result;
}

}}}   // ns
#endif