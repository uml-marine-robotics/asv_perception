#ifndef CLASSIFIED_OBSTACLE_2D_H
#define CLASSIFIED_OBSTACLE_2D_H

#include <asv_perception_common/Classification.h>
#include <asv_perception_common/Obstacle.h>

#include "defs.h"
#include "Homography.h"

namespace obstacle_id {
namespace detail {
namespace impl {

    using namespace asv_perception_common;

    // projects a roi to a vector of points using homography
    inline std::vector<geometry_msgs::Point32> roi_to_polygon( 
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
        asv_perception_common::Obstacle project( const Homography& h ) const {

            // default obstacle real world height, in world units
            static const float OBSTACLE_RW_HEIGHT_DEFAULT = 1.f;

            asv_perception_common::Obstacle result = {};

            result.label = this->cls.label;
            result.probability = this->cls.probability;
            
            // todo:  compute z offset based on parent(s)
            const float z_offset = 0.f;
            
            // real-world height in world units
            //  todo:  estimate real world height based on img bb, location
            const auto height = OBSTACLE_RW_HEIGHT_DEFAULT;

            // estimate a real-world depth based on height
            const auto depth = height;
            
            // create points; first point connects to last point
            result.shape.points = impl::roi_to_polygon( this->cls.roi, depth, height, z_offset, h );

            // compute centroid for pose.position
            const auto minmax = utils::minmax_3d( result.shape.points );

            result.pose.position.x = ( minmax.first.x + minmax.second.x ) / 2.;
            result.pose.position.y = ( minmax.first.y + minmax.second.y ) / 2.;
            result.pose.position.z = ( minmax.first.z + minmax.second.z ) / 2.;

            return result;

        }   // project
};  // class

}}   // ns
#endif