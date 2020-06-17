#include "test_common.h"
#include <gtest/gtest.h>

#define TEST_CASE_NAME TestHomography

namespace {
    using namespace obstacle_id;
    using namespace obstacle_id::detail;
}

// homography test
TEST( TEST_CASE_NAME, homography )
{   
    const auto h = ::getRGBtoWorldHomography();

    // transforms validated with calibrate.py
    //  rgb image to real world
    //  150, 400 --> -11, 22
    //  200, 375 --> -14, 33
    const auto r1 = h( 150.f, 400.f );
    EXPECT_EQ( (int)r1.first, -11 );
    EXPECT_EQ( (int)r1.second, 22 );

    const auto r2 = h( 250.f, 400.f );
    EXPECT_EQ( (int)r2.first, -8 );
    EXPECT_EQ( (int)r2.second, 21 );

}

TEST( TEST_CASE_NAME, inverse )
{
    const auto inv = ::getRGBtoWorldHomography().inverse();

    const auto pt = inv(0,0);

    // validated on wolfram alpha
    EXPECT_EQ((int)pt.first, 1067);
    EXPECT_EQ((int)pt.second, 1887);
}

TEST( TEST_CASE_NAME, overflow )
{
    const auto 
        h = ::getRGBtoWorldHomography().inverse()
        ;
    
    const auto pt = h( std::numeric_limits<float>::max(), std::numeric_limits<float>::max() );

    EXPECT_TRUE( std::isnormal( pt.first ) );
    EXPECT_TRUE( std::isnormal( pt.second ) );
}