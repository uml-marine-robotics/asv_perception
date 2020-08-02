#ifndef CLASSIFIED_OBSTACLE_2D_H
#define CLASSIFIED_OBSTACLE_2D_H

#include <asv_perception_common/Classification.h>
#include <asv_perception_common/Obstacle.h>

#include "defs.h"
#include "Homography.h"
#include "obstacle_extraction.h"

namespace obstacle_id {
namespace detail {
namespace impl {

    using namespace asv_perception_common;

    // projects a roi to a point cloud using homography
    //  rep-105/right hand rule for depth, width, height
    inline typename pointcloud_type::Ptr roi_to_pointcloud( 
        const sensor_msgs::RegionOfInterest& roi
        , const float depth
        , const float height
        , const float z_offset
        , const Homography& h 
        ) 
    {
        const auto make_point = []( float x, float y, float z ) { 
            return point_type(x,y,z);
        };  // make_point

        // construct 2 points at base of detection, other points are built from those
        //  blf = bottom-left-front, brf = bottom-right-front
        const auto 
            blf = h( roi.x_offset, roi.y_offset + roi.height )
            , brf = h( roi.x_offset + roi.width, roi.y_offset + roi.height )
            ;
        
        auto result = typename pointcloud_type::Ptr{ new pointcloud_type{} };

        // todo(?):  blf and brf x values may differ significantly (eg, 80+ m) as bounding boxes approach horizon
        //  average them or something so that the two bottom points have the same x value, then ensure depth is at least some minimum

        // bottom 4 points, counter-clockwise from bottom left front
        result->push_back( make_point( blf.first, blf.second, z_offset ) );   // blf
        result->push_back( make_point( brf.first, brf.second, z_offset ) );   // brf
        result->push_back( make_point( brf.first + depth, brf.second, z_offset ) );   // brr
        result->push_back( make_point( blf.first + depth, blf.second, z_offset ) );   // blr

        // top 4 points, clockwise from top left rear
        result->push_back( make_point( blf.first + depth, blf.second, z_offset + height ) );   // tlr
        result->push_back( make_point( brf.first + depth, brf.second, z_offset + height ) );   // trr
        result->push_back( make_point( brf.first, brf.second, z_offset + height ) );   // trf
        result->push_back( make_point( blf.first, blf.second, z_offset + height ) );   // tlf

        return result;
    }   // roi_to_pointcloud
} // impl

// represents a 2d obstacle defined by a classification message
class ClassifiedObstacle2d {

    public:

        asv_perception_common::Classification cls;
        std::weak_ptr<ClassifiedObstacle2d> parent;

        ClassifiedObstacle2d() = default;
        ClassifiedObstacle2d( asv_perception_common::Classification cls )
            : cls(std::move(cls))
            , parent()
        {}

        // projects to 3D using homography and creates Obstacle message 
        asv_perception_common::Obstacle project( 
            const Homography& h
            , const float min_height
            , const float max_height
            , const float min_depth
            , const float max_depth
        ) const {
            
            // todo:  compute z offset based on parent(s)
            const float z_offset = 0.f;
            
            // real-world height in world units
            //  todo:  estimate real world height based on img bb, location
            const auto height = max_height;

            // estimate a real-world depth
            const auto depth = height;
            
            // project roi to pointcloud
            auto pc_ptr = impl::roi_to_pointcloud( this->cls.roi, depth, height, z_offset, h );

            auto result = obstacle_extraction::impl::create_obstacle( pc_ptr );

            result.label = this->cls.label;
            result.label_probability = this->cls.probability;

            return result;

        }   // project
};  // class

}}   // ns
#endif