#ifndef OBSTACLE_PROJECTION2D_H
#define OBSTACLE_PROJECTION2D_H

#include "defs.h"
#include "utils.h"
#include "Obstacle2d.h"

namespace obstacle_id {
namespace obstacle_projection2d {

namespace impl {

    // default pointcloud label value
    static const int LABEL_DEFAULT = 128;

    // projects 2d shape to pointcloud
    void project_shape( 
        const cv::Rect& shape
        , const homography_type& homography
        , const int label
        , const int max_height
        , const int max_depth
        , pointcloud_type& pc 
    ) {


    }   // project_shape
}

// project obstacles in 2d image to provided pointcloud
void project( const image_type&, const homography_type&, pointcloud_type& );

// project obstacles in 2d bounding boxes to provided pointcloud
inline void project( 
    const std::vector<std::shared_ptr<Obstacle2d>>& obstacles
    , const homography_type& homography
    , const classification_pixel_map_type& pixel_map
    , const int max_height
    , const int max_depth
    , pointcloud_type& pc 
) {

    // generate points for each obstacle, place into provided pointcloud
    for ( const auto& obs_ptr : obstacles ) {
        if ( !obs_ptr )
            continue;
        
        // label lookup in pixel_map
        auto label = impl::LABEL_DEFAULT;
        if ( auto it = pixel_map.find( obs_ptr->cls.label ) )
            label = it->second;
        
        impl::project_shape( 
            utils::to_cv_rect( obs_ptr->cls.roi )
            , homography, label, max_height, max_depth, pc 
        );
    };  // for

}   // project

}}   // ns
#endif