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

    // create n-dimensional convex hull from pointcloud
    inline pointcloud_type create_convex_hull( 
        const typename pointcloud_type::Ptr& pc_ptr
        , const int ndimensions = 2 ) {

        pcl::ConvexHull<point_type> chull = {};
        chull.setInputCloud( pc_ptr );

        chull.setDimension( ndimensions );

        pointcloud_type hull = {};
        chull.reconstruct( hull );

        return hull;
    }   // create_convex_hull

    // create Obstacle from points and indices
    inline asv_perception_common::Obstacle create_obstacle( 
        const typename pointcloud_type::Ptr& pc_ptr
        , const pcl::PointIndices& pi
        ) {

        assert( !pi.indices.empty() );

        // getting segfaults when trying to use convexhull with setIndices. only for organized pointcloud?
        //  make a new pointcloud based on provided pointindices

        asv_perception_common::Obstacle result = {};
        
        // copy points from src pointcloud
        pointcloud_type::Ptr pc_copy_ptr( new pointcloud_type() );
        pcl::copyPointCloud( *pc_ptr, pi, *pc_copy_ptr );

        const auto minmax = utils::minmax_3d( pc_copy_ptr->points );

        result.dimensions.x = minmax.second.x - minmax.first.x;
        result.dimensions.y = minmax.second.y - minmax.first.y;
        result.dimensions.z = minmax.second.z - minmax.first.z;
        
        result.pose.pose.position.x = ( minmax.first.x + minmax.second.x ) / 2.;
        result.pose.pose.position.y = ( minmax.first.y + minmax.second.y ) / 2.;
        result.pose.pose.position.z = ( minmax.first.z + minmax.second.z ) / 2.;

        // 2d convex hull
        result.points = {};
        pcl::toROSMsg( create_convex_hull( pc_copy_ptr, 2 ), result.points );
        
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
        results.emplace_back(
            impl::create_obstacle( pc_ptr, cluster )
        );
    }

    return results;
}   // extract

}}} // ns

#endif