#ifndef OBSTACLE_PROJECTION_H
#define OBSTACLE_PROJECTION_H

#include <asv_perception_common/Obstacle.h>

#include "defs.h"
#include "utils.h"
#include "detail/Homography.h"

namespace obstacle_id {
namespace detail {
namespace obstacle_projection {

/*
Description of img pixels -> pointcloud projection algorithm: 
    Given a single channel image in 2D, iterate horizontally and process vertical scanlines
    For each vertical scanline:
        Starting from the bottom, project each scanline's obstacle pixels using homography
        Contiguous pixels are treated as a group, projected at the base pixel
        Obstacles must start below horizon, but can extend above the horizon (height)
*/

namespace impl {

    // compute the image row (y) value which approximates the horizon at image column (x)
    inline std::uint32_t compute_rgb_horizon( const std::uint32_t x, const Homography& rgb_to_radar ) {
        // round up
        return (std::uint32_t)std::round(
             ( -( (float)x * rgb_to_radar.at( 2, 0 ) + rgb_to_radar.at(2,2) ) / rgb_to_radar.at(2,1) )
             + 0.5f
        );
    }   // compute_rgb_horizon

    // appends points to pointcloud at x and y; constrained to x + height and y + depth at resolution (interval)
    inline void append_points(
        const float x
        , const float y
        , const float max_height
        , const float max_depth
        , const float resolution
        , pointcloud_type& pc
    )
    {
        assert( resolution > 0.f );
        assert( max_height > 0.f );
        assert( max_depth > 0.f );

        float z = 0.f;
        do {
            float y_offset = 0.f;
            do {

                point_type pt = {};
                pt.x = (decltype(pt.x))x;
                pt.y = (decltype(pt.y))( y + y_offset );
                pt.z = (decltype(pt.z))z;
                pc.push_back( pt );

                y_offset += resolution;
            } while ( y_offset <= max_depth );

            z += resolution;
        } while ( z <= max_height );
    }

    // projects obstacle pixels within vertical scanline at img column x to pointcloud
    inline void project_scanline( 
        const std::uint32_t x
        , const image_type& img
        , const Homography& rgb_to_radar
        , const float max_height
        , const float max_depth
        , const float resolution
        , const float max_distance
        , pointcloud_type& pc ) 
    {
        const auto max_distance_squared = ( max_distance > 0.f ) ? (float)std::pow( max_distance, 2.f ) : 0.f;

        int start_y = -1;  // current obstacle start y pixel; -1 for no current obstacle

        // at the current x and start_y, create an obstacle
        //  param is placeholder for future height estimation: using start_y - (param)
        const auto transform_append = [&]( int ) -> bool {
            const auto world_pt = rgb_to_radar( (float)x, (float)start_y );

            // do not create any obstacles with distance > max
            if (
                max_distance > 0.f
                && ( ( std::pow( world_pt.first, 2.f ) + std::pow( world_pt.second, 2.f ) ) > max_distance_squared )
                )
                return false;

            // todo:  height/depth estimation.  for now, use max values
            append_points( world_pt.first, world_pt.second, max_height, max_depth, resolution, pc );
            return true;
        };

        for ( int y = img.rows - 1, horizon = (int)compute_rgb_horizon( x, rgb_to_radar ); y > horizon; --y ) {

            if ( img.at<std::uint8_t>( y, (int)x ) > 0 ) {   // pixel has obstacle class
                if ( start_y < 0 ) // not currently in an obstacle
                    start_y = y;   // set start pixel for new obstacle
                // else, is contiguous obstacle pixel; continue
            } else if ( start_y >= 0 )  { // pixel does not have obstacle class, but we had one before
                const bool success = transform_append( y );
                start_y = -1;
                if ( !success ) // beyond max_distance, exit loop
                    break;
            }
        }

        // handle any remaning obstacles
        if ( start_y >= 0 )
            transform_append( 0 );
    }   // project_scanline

}   // impl

/*
Projects obstacles in a single channel obstacle img to point cloud
    any pixels > 0 in the map are considered obstacles
*/
inline pointcloud_type project( 
    const image_type& img
    , const Homography& rgb_to_radar
    , const float max_height = 1.f
    , const float max_depth = 1.f
    , const float resolution = 0.25f
    , const float max_distance = 100.f
)
{
    assert( img.channels() == 1 );
    assert( img.rows >= 0 );
    assert( img.cols >= 0 );
    
    pointcloud_type result = {};

    for ( int x = 0; x < img.cols; ++x )
        impl::project_scanline( x, img, rgb_to_radar, max_height, max_depth, resolution, max_distance, result );

    return result;
}


}}}   // ns
#endif