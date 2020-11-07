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

namespace impl {

    // realigns vertical roi coordinates to fit obstacle map; returns new roi
    inline sensor_msgs::RegionOfInterest adjust_vertical_roi_to_image( 
        const sensor_msgs::RegionOfInterest& roi
        , const image_type& img ) {

        // return flag if an obstacle pixel exists at horizontal scanline y between roi x_offset and x_offset+width
        const auto obstacle_pixel_at_y = [&]( int y ) {
            for ( int x = (int)roi.x_offset; x < ( int( roi.x_offset + roi.width ) ); ++x ) {
                if ( img.at<std::uint8_t>( y, x ) > 0 )
                    return true;
            }
            return false;
        };

        // loop while condition holds, or y != limit.  move y towards limit
        const auto find_stop_y = [&]( int y, const bool expected_condition_result, const int limit ) {

            auto result = y;

            while ( obstacle_pixel_at_y( y ) == expected_condition_result ) {
                result = y;

                if ( y > limit )
                    y -= 1;
                else if ( y < limit )
                    y += 1;
                else  // y==limit
                    break;
            }

            return result;
        };

        // find new top of roi
        //  if pixel at this y, expand up until no more pixels found
        //      if no pixel at this y, expand down until pixel found
        int start_y = (int)roi.y_offset;
        if ( obstacle_pixel_at_y( start_y ) )
            start_y = find_stop_y( start_y, true, 0 );
        else {
            start_y = find_stop_y( start_y, false, img.rows - 1 );
            start_y = std::min( start_y + 1, img.rows - 1); // add 1 to set location to where pixels exist
        }
        
        // find new bottom of roi
        int stop_y = int(roi.y_offset+roi.height);
        if ( obstacle_pixel_at_y( stop_y ) )  // move down until no more pixels found
            stop_y = find_stop_y( stop_y, true, img.rows - 1 );
        else {    // move up until pixels found
            stop_y = find_stop_y( stop_y, false, 0 );
            stop_y = std::max( stop_y - 1, 0 ); // subtract 1 to set location to where pixels exist
        }

        // sanity check
        if ( stop_y <= start_y )
            return roi;

        auto result = roi;
        result.y_offset = (decltype(result.y_offset))start_y;
        result.height = (decltype(result.height))( stop_y - start_y + 1 );
        return result;
    }

    // performs roi adjustments on obstacle roi bounding box using obstacle map
    //  returns flag if roi adjusted
    bool adjust_roi( 
        const std::shared_ptr<ClassifiedObstacle2d>& obs2d
        , const std::vector<std::shared_ptr<ClassifiedObstacle2d>>& obstacles_2d
        , const image_type& obstacle_map
        , const float roi_grow_limit
        , const float roi_shrink_limit 
        ) {
        
        // vertically expand/contract bounding boxes
        //  make roi adjustments up to min/max percentage

        //  do not adjust obstacle roi if overlapping another obstacle roi
        for ( const auto& other_obs2d : obstacles_2d ) {
            
            if ( obs2d.get() == other_obs2d.get() ) // same obj
                continue;

            if ( obs2d->roi_intersection_area( *other_obs2d ) > 0.f )
                return false;
        }

        auto candidate_roi = impl::adjust_vertical_roi_to_image( obs2d->cls.roi, obstacle_map );

        // compute roi area delta, compare to limits
        const auto 
            orig_area = obs2d->cls.roi.width * obs2d->cls.roi.height
            , candidate_area = candidate_roi.width * candidate_roi.height
            ;
        // delta is percent of original roi
        const auto delta = float(candidate_area) / float(std::max( (int)orig_area, 1 ));

        if ( 
            ( ( delta > 1.f ) && ( roi_grow_limit >= ( delta - 1.f ) ) )    // roi has grown
            || ( ( delta < 1.f ) && ( roi_shrink_limit >= ( 1.f - delta ) ) )   // roi has shrunk
        ) {
            obs2d->cls.roi = std::move( candidate_roi );
            return true;
        }
        return false;
        
    } // adjust_roi

    // returns flag if base of classified obstacle bounding box exists above the horizon
    inline bool is_above_horizon( const ClassifiedObstacle2d& c, const Homography& h ) {
        
        // get horizon location at pixel x
        const auto horizon_at_x = [&h]( const std::uint32_t x ) {
            return std::uint32_t( std::max( -( float(x)*h.at(2,0) + h.at(2,2))/ ( h.at(2,1) + FLT_EPSILON ), 0.f ) );
        };

        const auto& roi = c.cls.roi;
        // ROS_WARN_STREAM( std::to_string( horizon_at_x( roi.x_offset ) ) + " / " + std::to_string( horizon_at_x( roi.x_offset + roi.width ) ) );            

        return
            ( ( roi.y_offset + roi.height ) <= horizon_at_x( roi.x_offset ) )
            || ( ( roi.y_offset + roi.height ) <= horizon_at_x( roi.x_offset + roi.width ) )
        ;
    }   // is_above_horizon

}   // impl

/*
Combines an unclassified obstacle map (optional) with a vector of classification bounding boxes
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
    , const float min_distance
    , const float max_distance
    , const float roi_grow_limit = 0.f
    , const float roi_shrink_limit = 0.f
)
{
    // if obstacle map provided, check inputs
    if ( !obstacle_map.empty() && ( classifications.image_height != obstacle_map.rows || classifications.image_width !=  obstacle_map.cols ) )
        throw std::runtime_error("obstacle map image size does not match classification image size");

    // foreach classification, construct Obstacle2d
    auto obstacles_2d = std::vector<std::shared_ptr<ClassifiedObstacle2d>>();
    for ( const auto& cls : classifications.classifications )
        obstacles_2d.emplace_back( std::make_shared<ClassifiedObstacle2d>(cls) );

    //  todo:  estimate z offset via parent/child relationships
    //      depends on adjacent pixel class, known/unknown
    //      eg assume boat and person are known classes.  
    //      if person located on top of boat, z offset = boat height?
    //      if person located on top of unknown class, is person at height of unknown class, 
    //          or does the person's bounding box need expansion?  other solution?

    // now project each to obstacle msg, remove bb from unknown obstacle map
    auto result = std::vector<Obstacle>{};
    const auto 
        max_distance_squared = std::pow( max_distance, 2.f )
        , min_distance_squared = std::pow(min_distance, 2.f )
        ;

    for ( auto& obs2d : obstacles_2d ) {

        // vertically expand/contract classifier roi bounding boxes
        //  make roi adjustments up to min/max percentage
        if ( !obstacle_map.empty() && ( ( roi_grow_limit > 0.f ) || ( roi_shrink_limit > 0.f ) ) )
            impl::adjust_roi( obs2d, obstacles_2d, obstacle_map, roi_grow_limit, roi_shrink_limit );

        if ( impl::is_above_horizon( *obs2d, h ) ) {
            ROS_INFO_STREAM( std::string( "Image ROI above horizon, ignoring" ) );
            continue;
        }

        auto obs = obs2d->project( h, min_height, max_height, min_depth, max_depth );

        // min/max distance check, if supplied
        if ( ( min_distance > 0.f ) || ( max_distance > 0.f ) ) {
            const auto dist_squared = 
                std::pow( obs.pose.pose.position.x, 2.f ) 
                + std::pow(obs.pose.pose.position.y, 2.f )
                + std::pow(obs.pose.pose.position.z, 2.f )
            ;

            if (
                ( dist_squared < min_distance_squared )
                || ( dist_squared > max_distance_squared )
                )
                continue;
        }

        if ( !obstacle_map.empty() ) {
            const auto rect = utils::to_cv_rect( obs2d->cls.roi, obstacle_map );    // get opencv rect based on roi
            obstacle_map( rect ) = 0;    // set roi to black in obstacle_map
        }

        result.emplace_back( std::move(obs) );
    }

    return result;
}

}}}   // ns
#endif