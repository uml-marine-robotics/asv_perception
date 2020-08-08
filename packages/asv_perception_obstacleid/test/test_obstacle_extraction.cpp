#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include "../include/detail/obstacle_projection.h"
#include "../include/detail/obstacle_extraction.h"
#include "test_common.h"

#define TEST_CASE_NAME TestObstacleExtraction
namespace {
    using namespace obstacle_id;
    using namespace obstacle_id::detail;
}

TEST( TEST_CASE_NAME, obstacle_extraction_basic )
{   
    // use obstacle_projection to create a set of points, then extraction to verify
    //  todo:  make 3d shape so we can generate/test convex hull

    pointcloud_type pc = {};

    const auto 
        x = 5.f
        , y = 5.f
        , resolution = 0.49f
        , max_depth = 0.5f
        , max_height = 1.f
        ;
    const auto
        depth_pts = int( max_depth / resolution ) + 1 // depth pts expected
        , height_pts = int( max_height / resolution ) + 1  // height pts expected
        ;

    obstacle_projection::impl::append_points( x, y, max_height, max_depth, resolution, pc );

    EXPECT_EQ( pc.size(), depth_pts * height_pts );
    const auto mm = utils::minmax_3d( pc.points );
    
    const auto min = mm.first;
    const auto max = mm.second;

    // pcl algos require shared_ptr
    const auto pc_ptr = typename pointcloud_type::Ptr( new pointcloud_type( pc ) );
    const auto obstacles = obstacle_extraction::extract( pc_ptr, 10.f, 1, std::numeric_limits<std::uint32_t>::max(), -0.f );

    ASSERT_EQ( obstacles.size(), 1 );
    const auto obs0 = obstacles.front();

    EXPECT_DOUBLE_EQ( obs0.pose.pose.position.x, ( min.x + max.x ) / 2. );
    EXPECT_DOUBLE_EQ( obs0.pose.pose.position.y, ( min.y + max.y ) / 2. );
    EXPECT_DOUBLE_EQ( obs0.pose.pose.position.z, ( min.z + max.z ) / 2. );

}

