// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#ifndef OBSTACLEID_POINT_CLUSTER_H
#define OBSTACLEID_POINT_CLUSTER_H

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <asv_perception_common/Obstacle.h>

#include "utils.h"

namespace obstacle_id {
namespace detail {

// Represents a cluster of points, and related info
class PointCluster {
public:

    typename pointcloud_type::Ptr src_ptr;
    pcl::PointIndices indices;
    pointcloud_type convex_hull;
    double area;

    // create pointcluster from pointcloud, indices
    PointCluster( typename pointcloud_type::Ptr& pc_ptr, pcl::PointIndices indices, const std::uint32_t ndimensions = 2 )
        : src_ptr{pc_ptr}
        , indices{std::move(indices)}
        , convex_hull{}
        , area{}
     {
        // compute hull, area/volume
        //  may fail if not enough dimensionality in x/y (ie, min x/y == max x/y), but does not throw
        auto hull_area = utils::create_convex_hull( *pc_ptr, this->indices, ndimensions );
        this->convex_hull = std::move(hull_area.first);
        this->area = hull_area.second;
    }

    // create Obstacle from this PointCluster
    asv_perception_common::Obstacle to_obstacle() const {

        asv_perception_common::Obstacle result = {};

        const auto minmax = utils::minmax_3d( *(this->src_ptr), this->indices );

        result.dimensions.x = minmax.second.x - minmax.first.x;
        result.dimensions.y = minmax.second.y - minmax.first.y;
        result.dimensions.z = minmax.second.z - minmax.first.z;
        
        // use weighted centroid for position
        result.pose.pose.position = utils::to_ros_point( utils::centroid( *(this->src_ptr), this->indices ) );
        result.pose.pose.orientation.w = 1.;

        result.area = this->area;

        // create convex hull points; relative to centroid
        result.hull2d = {};
        const auto& centroid = result.pose.pose.position;
        for ( const auto& hull_pt : this->convex_hull.points ) {
            geometry_msgs::Point32 pt = {};
            pt.x = centroid.x - hull_pt.x;
            pt.y = centroid.y - hull_pt.y;
            pt.z = centroid.z - hull_pt.z;
            result.hull2d.points.emplace_back( std::move( pt ) );
        }
        
        return result;
    }   // to_obstacle
    
    // extracts vector of PointCluster from the input pointcloud
    static std::vector<PointCluster> extract( 
        typename pointcloud_type::Ptr& pc_ptr
        , const float cluster_tolerance
        , const std::uint32_t min_cluster_sz
        , const std::uint32_t max_cluster_sz
        , const float min_area
        , const float max_area
        , const std::uint32_t ndimensions = 2
    ) {
        // do extraction from pointcloud
        pcl::search::Search<point_type>::Ptr tree( new pcl::search::KdTree<point_type>( false ) );
        tree->setInputCloud( pc_ptr );

        std::vector<pcl::PointIndices> clusters = {};
        pcl::extractEuclideanClusters( *pc_ptr, tree, cluster_tolerance, clusters, min_cluster_sz, max_cluster_sz );

        std::vector<PointCluster> result = {};

        for ( auto& cluster : clusters ) {

            if ( cluster.indices.empty() )
                continue;

            auto pcluster = PointCluster( pc_ptr, std::move( cluster ), ndimensions );

            if ( ( pcluster.area >= min_area ) && ( pcluster.area <= max_area ) )
                result.emplace_back(std::move(pcluster));
        }
            
        return result;
    }  // extract

};  //PointCluster


}}  // ns
#endif