#ifndef DETAIL_OBSTACLE_EXTRACTION_H
#define DETAIL_OBSTACLE_EXTRACTION_H

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/surface/convex_hull.h>    // convexHull
// #include <pcl/common/centroid.h>    // compute3DCentroid
#include <pcl_conversions/pcl_conversions.h>

#include <asv_perception_common/Obstacle.h>

#include "defs.h"
#include "utils.h"

namespace obstacle_id {
namespace detail {
namespace obstacle_extraction {

namespace impl {

    // create Obstacle from points and indices
    inline asv_perception_common::Obstacle create_obstacle( 
        const pointcloud_type& pc
        , const pcl::PointIndices& pi
        ) {

        assert( !pi.indices.empty() );

        // getting segfaults when trying to use convexhull with setIndices. only for organized pointcloud?
        //  make a new pointcloud based on provided pointindices
        //  also, convex hull is slow; just populate obstacle points.  consumer can compute if needed.  2d faster?

        asv_perception_common::Obstacle result = {};
        
        // copy points
        pointcloud_type pc_copy = {};
        pcl::copyPointCloud( pc, pi, pc_copy );

        const auto minmax = utils::minmax_3d( pc_copy.points );

        result.dimensions.x = minmax.second.x - minmax.first.x;
        result.dimensions.y = minmax.second.y - minmax.first.y;
        result.dimensions.z = minmax.second.z - minmax.first.z;
        
        result.pose.pose.position.x = ( minmax.first.x + minmax.second.x ) / 2.;
        result.pose.pose.position.y = ( minmax.first.y + minmax.second.y ) / 2.;
        result.pose.pose.position.z = ( minmax.first.z + minmax.second.z ) / 2.;

        result.points = sensor_msgs::PointCloud2();
        pcl::toROSMsg ( pc_copy, result.points );   // this is another copy
        
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
            impl::create_obstacle( *pc_ptr, cluster )
        );
    }

    return results;
}   // extract

}}} // ns

#endif