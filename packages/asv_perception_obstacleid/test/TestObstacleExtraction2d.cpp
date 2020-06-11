#include <iostream>
#include <asv_perception_common/ClassificationArray.h>
#include "../include/utils.h"
#include "../include/obstacle_extraction2d.h"
#include <gtest/gtest.h>

#define TEST_CASE_NAME TestObstacleExtraction2d
namespace {
    using namespace obstacle_id;

    // image (1280x1024) to world matrix (radar img:  1024x1024, 220m real-world diameter)
    static const std::vector<float> HOMOGRAPHY_DATA = { 
        0.47386, -0.113292, -291.825,
        0.053203, -0.387605, 674.903,
        0.0163188, 0.311054, -103.878
    };

}

// homography test
TEST( TEST_CASE_NAME, homography )
{   
    const auto h = Homography( HOMOGRAPHY_DATA );

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

TEST( TEST_CASE_NAME, obstacle_extraction2d_basic )
{   
    const int PX_VAL = 255;
    cv::Mat img( 1024, 1280, CV_8U,cv::Scalar(PX_VAL));

    asv_perception_common::ClassificationArray v = {};
    v.image_width = img.cols;
    v.image_height= img.rows;
    
    auto c = asv_perception_common::Classification() ;

    // match base points from above:  150,400; 250,400
    c.roi.height = 50;
    c.roi.x_offset = 150;
    c.roi.width = 250 - c.roi.x_offset;
    c.roi.y_offset = 400 - c.roi.height;  // height + y_offset = base y
    
    c.label = "my_obstacle";
    c.probability = 0.5;
    v.classifications.emplace_back( c );

    const auto r = obstacle_extraction2d::create_obstacle2d( img, v );

    ASSERT_TRUE( !r.empty() );
    ASSERT_TRUE( r.front() );// check pointer

    // get the roi of the classification from the image, ensure it's been cleared    
    const auto cv_roi = utils::to_cv_rect( r.front()->cls.roi );
    EXPECT_EQ( cv::sum( img( cv_roi ) )[0], 0 );

    // check projection
    const auto h = Homography( HOMOGRAPHY_DATA );
    const auto projs = obstacle_extraction2d::project_obstacle2d( r, h );

    ASSERT_TRUE( !projs.empty() );
    const auto proj0 = projs[0];
    EXPECT_EQ( proj0.label, c.label );
    EXPECT_DOUBLE_EQ( proj0.probability, c.probability );
    EXPECT_EQ( proj0.shape.points.size(), 8 );

    // first point is bottom-left-front, second point is bottom-right-front.  check vs verified homography in previous test
    EXPECT_EQ( (int)proj0.shape.points[0].x, -11 );
    EXPECT_EQ( (int)proj0.shape.points[0].y, 22 );
    EXPECT_EQ( (int)proj0.shape.points[0].z, 0 );
    
    EXPECT_EQ( (int)proj0.shape.points[1].x, -8 );
    EXPECT_EQ( (int)proj0.shape.points[1].y, 21 );
    EXPECT_EQ( (int)proj0.shape.points[1].z, 0 );

}