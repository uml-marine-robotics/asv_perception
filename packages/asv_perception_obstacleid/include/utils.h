#ifndef OBSTACLEID_UTILS_H
#define OBSTACLEID_UTILS_H

//#include <pcl/kdtree/kdtree.h>
//#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>    // convexHull
#include <pcl/common/common.h>  // minMax3D
#include <pcl/common/centroid.h>    // compute3DCentroid

#include <sensor_msgs/RegionOfInterest.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "defs.h"

namespace obstacle_id {
namespace utils {

// convert point type to ros point of type R
template <typename R=geometry_msgs::Point>
inline R to_ros_point( const point_type& pt ) {
    R result = {};
    result.x = pt.x;
    result.y = pt.y;
    result.z = pt.z;
    return result;
}

// convert vector to point_type
inline point_type to_point( const Eigen::Vector4f& v ) {
    point_type result = {};
    result.x = v[0];
    result.y = v[1];
    result.z = v[2];
    return result;
}

template <typename T>
inline T ros_tf_transform( const T& what, const std::string& from, const std::string& to, const int time = 0, const float duration = 1.0 ) {

    tf2_ros::Buffer tf_buffer = {};
    tf2_ros::TransformListener tf2_listener(tf_buffer);
    const auto lookup = tf_buffer.lookupTransform( to, from, ros::Time( time ), ros::Duration( duration ) );

    T result = {};
    tf2::doTransform( what, result, lookup );// transform 'what' to 'result'
    return result;
}   // ros_tf_transform

// convert ros roi to opencv rect, clamped to image shape
inline cv::Rect to_cv_rect( const sensor_msgs::RegionOfInterest& roi, const image_type& img ) {
    
    cv::Rect result = {}; // cv::Rect( (int)roi.x_offset, (int)roi.y_offset, (int)roi.width, (int)roi.height );
    if ( img.empty() )
        return result;

    result.x = std::max( std::min( (int)roi.x_offset, img.cols - 1), 0 );
    result.y = std::max( std::min( (int)roi.y_offset, img.rows - 1), 0 );
    result.width = std::max( std::min( (int)roi.width, img.cols - result.x ), 0 );
    result.height = std::max( std::min( (int)roi.height, img.rows - result.y ), 0 );
    return result;
}

// return minmax of pointcloud and indices
inline std::pair<point_type, point_type> minmax_3d( const pointcloud_type& pc, const pcl::PointIndices& indices ) {

    Eigen::Vector4f 
        minpt = {}
        , maxpt = {}
        ;

    pcl::getMinMax3D ( pc, indices.indices, minpt, maxpt );

    //return std::make_pair<point_type, point_type>( to_point( std::move( minpt ) ), to_point( std::move( maxpt )) );
    return { to_point( std::move( minpt ) ), to_point( std::move( maxpt ) ) };
}

// based on https://stackoverflow.com/q/35669182/882436
template <typename PointT, typename Alloc>
inline std::pair<PointT, PointT> minmax_3d( const std::vector<PointT, Alloc>& points ) {

    if ( points.empty() )
        return {};

    auto
        min = points.front()
        , max = points.front()
        ;

    for ( const auto& pt : points ) {
        min.x = std::min( min.x, pt.x );
        min.y = std::min( min.y, pt.y );
        min.z = std::min( min.z, pt.z );
        max.x = std::max( max.x, pt.x );
        max.y = std::max( max.y, pt.y );
        max.z = std::max( max.z, pt.z );
    }

    return std::make_pair(std::move(min), std::move(max));
}

// compute centroid of pointcloud, indices
inline point_type centroid( const pointcloud_type& pc, const pcl::PointIndices& indices ) {
    
    Eigen::Vector4f 
        centroid = {}
        ;

    pcl::compute3DCentroid( pc, indices.indices, centroid );

    return to_point( centroid );
}

// test whether a PointCloud2 is valid, based on 
//  https://github.com/ros-perception/perception_pcl/blob/melodic-devel/pcl_ros/include/pcl_ros/pcl_nodelet.h
inline bool is_cloud_valid( const sensor_msgs::PointCloud2::ConstPtr& cloud ) {
    return 
        ( cloud )
        && ( cloud->width * cloud->height * cloud->point_step == cloud->data.size() )
        ;
}

// create/return n-dimensional convex hull from pointcloud indices and its area (2d) or volume (3d)
inline std::pair<pointcloud_type, double> create_convex_hull( 
    const pointcloud_type& pc
    , const pcl::PointIndices& indices
    , const int ndimensions = 2 
) {
    if ( indices.indices.empty() )
        return {};

    // copy cluster points from src pointcloud.  
    //    convex hull seems to require an organized pointcloud if providing indices?  otherwise, segfault
    pointcloud_type::Ptr cluster_pc( new pointcloud_type() );
    pcl::copyPointCloud( pc, indices, *cluster_pc );

    // if 2d, flatten the pointcloud so convexhull is created for x, y dimensions
    if ( ndimensions < 3 ) {
        const auto minmax = minmax_3d( cluster_pc->points );
        for ( auto& pt : cluster_pc->points )
            pt.z = minmax.first.z;
    }

    pcl::ConvexHull<point_type> chull = {};
    chull.setInputCloud( cluster_pc );
    chull.setComputeAreaVolume( true );
    chull.setDimension( ndimensions );

    pointcloud_type hull = {};
    chull.reconstruct( hull );

    return { std::move( hull ), ( ndimensions == 2 ) ? chull.getTotalArea() : chull.getTotalVolume() };
}   // create_convex_hull

// creates a pointcluster and computes attributes for a pointcloud and indices
/*
inline PointCluster create_point_cluster( typename pointcloud_type::Ptr& pc_ptr, pcl::PointIndices indices, const std::uint32_t ndimensions = 2 ) {

    PointCluster result = {};
    result.src_ptr = pc_ptr;
    result.indices = std::move(indices);

    // compute hull, area/volume
    //  may fail if not enough dimensionality in x/y (ie, min x/y == max x/y), but does not throw
    auto hull_area = create_convex_hull( *pc_ptr, result.indices, ndimensions );
    result.convex_hull = std::move(hull_area.first);
    result.area = hull_area.second;

    return result;
}   // create_point_cluster
*/

/*
// extracts vector of PointCluster from the input pointcloud
inline std::vector<PointCluster> extract_clusters( 
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

        auto pcluster = create_point_cluster( pc_ptr, std::move( cluster ), ndimensions );

        if ( ( pcluster.area >= min_area ) && ( pcluster.area <= max_area ) )
            result.emplace_back(std::move(pcluster));
    }
        
    return result;
 }  // extract_clusters
 */
}}  // ns
#endif