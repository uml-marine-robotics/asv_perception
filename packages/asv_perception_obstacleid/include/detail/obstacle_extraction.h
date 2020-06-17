#ifndef DETAIL_OBSTACLE_EXTRACTION_H
#define DETAIL_OBSTACLE_EXTRACTION_H

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>    // convexHull
#include <pcl/common/centroid.h>    // compute3DCentroid
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <asv_perception_common/Obstacle.h>

#include "defs.h"
#include "utils.h"

namespace obstacle_id {
namespace detail {
namespace obstacle_extraction {

namespace impl {

    // create Obstacle from points, append to vector on success. returns flag of success
    inline asv_perception_common::Obstacle create_obstacle( 
        const pointcloud_type& pc
        , const pcl::PointIndices& pi
        ) {

        assert( !pi.indices.empty() );

        // getting segfaults when trying to use convexhull with setIndices.
        //  instead, make a new pointcloud based on provided pointindices
        //  todo:  investigate.  only for organized pointcloud?
        //  also, convex hull is slow; just populate obstacle points.  consumer can compute if needed

        asv_perception_common::Obstacle result = {};
        
        // convert pointcloud pts to shape.points
        for ( const auto idx : pi.indices )
            result.shape.points.emplace_back( utils::to_point32( pc[idx] ) );

        // centroid to pose.position
        Eigen::Vector4f centroid_pt = {};
        pcl::compute3DCentroid( pc, pi, centroid_pt );
        result.pose.position.x = centroid_pt.x();
        result.pose.position.y = centroid_pt.y();
        result.pose.position.z = centroid_pt.z();

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
    , const float leaf_size
    , const float min_x_distance
    , const float min_y_distance
)
{
    // downsampling
    if ( leaf_size > 0.f ) {
        pcl::VoxelGrid<point_type> vg = {};
        vg.setInputCloud( pc_ptr );
        vg.setLeafSize( leaf_size, leaf_size, leaf_size );
        vg.filter( *pc_ptr );
    }

    // self return filter
    //  todo: refactor this + leaf size filter into separate nodelet

    // Filter out all points within threshold box from vehicle (origin)
    
    using PointType = point_type;
    pcl::ConditionOr<PointType>::Ptr range_cond (new pcl::ConditionOr<PointType> ());
    
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
                                    pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::GT, min_x_distance)));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
                                    pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::LT, -min_x_distance)));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
                                    pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::GT, min_y_distance)));
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
                                    pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::LT, -min_y_distance)));

    pointcloud_type::Ptr cloud_post_filter (new pointcloud_type);
    pcl::ConditionalRemoval<PointType> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(pc_ptr);
    condrem.filter(*cloud_post_filter);
    
    auto pc = cloud_post_filter;

    // do extraction from pointcloud
    pcl::search::Search<point_type>::Ptr tree( new pcl::search::KdTree<point_type> );
    tree->setInputCloud( pc );

    std::vector<pcl::PointIndices> clusters = {};
    pcl::extractEuclideanClusters( *pc, tree, cluster_tolerance, clusters, min_cluster_size, max_cluster_size );

    std::vector<asv_perception_common::Obstacle> results = {};

    // convert clusters to obstacles
    for ( const auto& cluster : clusters ) {
        if ( cluster.indices.empty() )
            continue;
        results.emplace_back(
            impl::create_obstacle( *pc, cluster )
        );
    }

    return results;
}   // extract

}}} // ns

#endif