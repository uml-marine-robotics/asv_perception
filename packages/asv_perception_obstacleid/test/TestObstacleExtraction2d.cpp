#include <iostream>
#include <asv_perception_common/ClassificationArray.h>
#include "../include/utils.h"
#include "../include/obstacle_extraction2d.h"
#include <gtest/gtest.h>

#define TEST_CASE_NAME TestObstacleExtraction2d
namespace {
    using namespace obstacle_id;

    // image (1280x1024) to radar (1024,1024) matrix
    static const std::vector<float> HOMOGRAPHY_DATA = { 
        2.268932,         34.102644,       -11718.441166,
        1.741869,         34.603541,       -12101.519025,
        0.003506,         0.066828,        -22.317610
    };

}

// homography test
TEST( TEST_CASE_NAME, homography )
{   
    const auto h = Homography( HOMOGRAPHY_DATA );

    // transforms validated with calibrate.py
    //  rgb image to radar
    //  512,512 --> 504,475
    const auto r1 = h( 512.f, 512.f );
    EXPECT_EQ( (int)r1.first, 504 );
    EXPECT_EQ( (int)r1.second, 475 );

}

TEST( TEST_CASE_NAME, basic )
{   
    
    const int PX_VAL = 255;
    cv::Mat img( 1024, 1280, CV_8U,cv::Scalar(PX_VAL));

    asv_perception_common::ClassificationArray v = {};
    v.image_width = img.cols;
    v.image_height= img.rows;
    
    auto c = asv_perception_common::Classification() ;
    c.roi.x_offset = 10;
    c.roi.y_offset = 10;
    c.roi.width = 20;
    c.roi.height = 30;
    v.classifications.emplace_back( c );

    auto r = obstacle_extraction2d::extract( img, v, Homography( HOMOGRAPHY_DATA ) );

    // todo:
    //  validate bb removed from img
    //  validate resulting obstacle

    ASSERT_TRUE(true);

}