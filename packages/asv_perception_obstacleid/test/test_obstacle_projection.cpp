// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#include <gtest/gtest.h>
#include "../include/detail/obstacle_projection.h"
#include "test_common.h"

#define TEST_CASE_NAME TestObstacleProjection
namespace {
    using namespace obstacle_id;
    using namespace obstacle_id::detail;
}

TEST( TEST_CASE_NAME, compute_rgb_horizon )
{
    const auto h = ::getRGBtoWorldHomography();

    EXPECT_EQ( obstacle_projection::impl::compute_rgb_horizon( 0, h ), 334 );
    EXPECT_EQ( obstacle_projection::impl::compute_rgb_horizon( 1280, h ), 267 );
    
}

TEST( TEST_CASE_NAME, append_points )
{   
    pointcloud_type pc = {};

    const auto 
        resolution = 0.49f
        , max_depth = 0.5f
        , max_height = 1.f
        ;
    const auto
        depth_pts = int( max_depth / resolution ) + 1 // depth pts expected
        , height_pts = int( max_height / resolution ) + 1  // height pts expected
        ;

    obstacle_projection::impl::append_points( 0.f, 0.f, max_height, max_depth, resolution, pc );

    EXPECT_EQ( pc.size(), depth_pts * height_pts );
    const auto mm = utils::minmax_3d( pc.points );
    
    const auto min = mm.first;
    const auto max = mm.second;

    EXPECT_FLOAT_EQ( min.x, 0.f );
    EXPECT_FLOAT_EQ( min.y, 0.f );
    EXPECT_FLOAT_EQ( min.z, 0.f );

    EXPECT_FLOAT_EQ( max.x, 0.f );
    EXPECT_FLOAT_EQ( max.y, ( depth_pts - 1 ) * resolution ); // max depth
    EXPECT_FLOAT_EQ( max.z, ( height_pts - 1 ) * resolution );  // max height

}

TEST( TEST_CASE_NAME, create_obstacles_basic )
{   
    const auto 
        h_rgb_world = ::getRGBtoWorldHomography()
        ;

    const int PX_VAL = 0;
    cv::Mat img( 1024, 1280, CV_8U,cv::Scalar(PX_VAL));

    const std::uint32_t 
        start_x = 1280 / 2
        , start_y = obstacle_projection::impl::compute_rgb_horizon( start_x, h_rgb_world ) + 20
        , w = 25
        , h = w
        ;


    // place obstacle at position below horizon
    auto rect = cv::Rect( start_x, start_y, w, h );
    img(rect) = 255;    // make non-zero pixels
    
    const auto pc = obstacle_projection::project( img, h_rgb_world );

    // check points
    const auto pt_exists = [&]( int x, int y, int z ) {

        const auto world_pt = h_rgb_world( x, y );
        for ( const auto& pt : pc.points ) {
            if ( 
                ( (int)pt.x == (int)world_pt.first ) 
                && ( (int)pt.y == (int)world_pt.second )
                 && ( (int)pt.z == z ) 
                )
                return true;
        }
        return false;
    };

    for ( int x_offset = 0; x_offset < w; x_offset++ )
        EXPECT_TRUE( pt_exists( start_x + x_offset, start_y + h - 1, 0 ) );
}

/*
TEST( TEST_CASE_NAME, from_image ) {

    const auto 
        h_rgb_world = ::getRGBtoWorldHomography()
        , h_radar_rgb = ::getRGBtoRadarHomography().inverse()
        ;

    auto img = cv::imread("packages/asv_perception_obstacleid/test/obstacle_projection.png", cv::IMREAD_GRAYSCALE );
    ASSERT_FALSE(img.empty());

    auto cloud = detail::obstacle_projection::project( img, h_rgb_world, h_radar_rgb );
}
*/