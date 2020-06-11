#ifndef OBSTACLE_2D_H
#define OBSTACLE_2D_H

#include <asv_perception_common/Classification.h>
#include <asv_perception_common/Obstacle.h>

#include "defs.h"
#include "Homography.h"

namespace obstacle_id {

// represents a 2d obstacle
class Obstacle2d {

    public:

        asv_perception_common::Classification cls;
        std::weak_ptr<Obstacle2d> parent;

        Obstacle2d() = default;
        Obstacle2d( asv_perception_common::Classification cls )
            : cls(std::move(cls))
            , parent()
        {}

        // converts to Obstacle using homography
        asv_perception_common::Obstacle to_obstacle( const Homography& h ) const {

            asv_perception_common::Obstacle result = {};

            result.label = this->cls.label;
            result.probability = this->cls.probability;
            
            // todo:  compute z offset based on parent(s)

            // compute polygon
            


            // compute centroid for pose.position

        }   // to_obstacle
};  // class

}   // ns
#endif