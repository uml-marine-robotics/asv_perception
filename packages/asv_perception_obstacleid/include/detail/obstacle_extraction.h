#ifndef DETAIL_OBSTACLE_EXTRACTION_H
#define DETAIL_OBSTACLE_EXTRACTION_H

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>    // convexHull
// #include <pcl/common/centroid.h>    // compute3DCentroid
#include <pcl_conversions/pcl_conversions.h>

#include <asv_perception_common/Obstacle.h>

#include "defs.h"
#include "utils.h"

namespace obstacle_id {
namespace detail {
namespace obstacle_extraction {

namespace impl {

    // create/return n-dimensional convex hull from pointcloud and its area (2d) or volume (3d)
    inline std::pair<pointcloud_type, double> create_convex_hull( 
        const typename pointcloud_type::Ptr& pc_ptr
        , const int ndimensions = 2 
    ) {

        pcl::ConvexHull<point_type> chull = {};
        chull.setInputCloud( pc_ptr );
        chull.setComputeAreaVolume( true );
        chull.setDimension( ndimensions );

        pointcloud_type hull = {};
        chull.reconstruct( hull );

        return { std::move( hull ), ( ndimensions == 2 ) ? chull.getTotalArea() : chull.getTotalVolume() };
    }   // create_convex_hull

    // create Obstacle from pointcloud
    inline asv_perception_common::Obstacle create_obstacle( 
        const pointcloud_type& pc
    ) {
        asv_perception_common::Obstacle result = {};
        const auto minmax = utils::minmax_3d( pc.points );

        result.dimensions.x = minmax.second.x - minmax.first.x;
        result.dimensions.y = minmax.second.y - minmax.first.y;
        result.dimensions.z = minmax.second.z - minmax.first.z;
        
        result.pose.pose.position.x = ( minmax.first.x + minmax.second.x ) / 2.;
        result.pose.pose.position.y = ( minmax.first.y + minmax.second.y ) / 2.;
        result.pose.pose.position.z = ( minmax.first.z + minmax.second.z ) / 2.;

        return result;
    }   // create_obstacle
}   // impl

/*
Extracts obstacles from point cloud
    input pointcloud may be modified
*/
inline std::vector<asv_perception_common::Obstacle> extract( 
    const typename pointcloud_type::Ptr& pc_ptr
    , const float cluster_tolerance
    , const std::uint32_t min_cluster_size
    , const std::uint32_t max_cluster_size
    , const float max_area
)
{
    if ( !pc_ptr || pc_ptr->empty() )
        return {};

    // do extraction from pointcloud
    pcl::search::Search<point_type>::Ptr tree( new pcl::search::KdTree<point_type>( false ) );
    tree->setInputCloud( pc_ptr );

    std::vector<pcl::PointIndices> clusters = {};
    pcl::extractEuclideanClusters( *pc_ptr, tree, cluster_tolerance, clusters, min_cluster_size, max_cluster_size );

    std::vector<asv_perception_common::Obstacle> results = {};

    // convert clusters to obstacles
    for ( const auto& cluster : clusters ) {
        
        if ( cluster.indices.empty() )
            continue;

        // copy cluster points from src pointcloud
        pointcloud_type::Ptr cluster_pc( new pointcloud_type() );
        pcl::copyPointCloud( *pc_ptr, cluster, *cluster_pc );

        auto hull_area = impl::create_convex_hull( cluster_pc );

        // max area check
        if ( ( max_area > 0.f ) && ( hull_area.second > max_area ) )
            continue;

        auto obs = impl::create_obstacle( hull_area.first );
        pcl::toROSMsg( hull_area.first, obs.points );
        results.emplace_back( std::move( obs ) );
    }

    return results;
}   // extract

}}} // ns

#endif