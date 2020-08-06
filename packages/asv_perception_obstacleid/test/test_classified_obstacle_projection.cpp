#include <iostream>
#include <asv_perception_common/ClassificationArray.h>
#include "../include/utils.h"
#include "../include/detail/classified_obstacle_projection.h"
#include "test_common.h"

#include <gtest/gtest.h>

#define TEST_CASE_NAME TestClassifiedObstacleProjection
namespace {
    using namespace obstacle_id;
    using namespace obstacle_id::detail;
}

TEST( TEST_CASE_NAME, create_obstacles_basic )
{   
    const auto h = ::getRGBtoWorldHomography();
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

    const auto projs = classified_obstacle_projection::project( img, v, h, 1.f, 1.f, 0.1f, 1.f, -0.f );

    // get the roi of the classification from the image, ensure it's been cleared    
    const auto cv_roi = utils::to_cv_rect( c.roi, img );
    EXPECT_EQ( cv::sum( img( cv_roi ) )[0], 0 );

    // check projection
    ASSERT_TRUE( !projs.empty() );
    const auto proj0 = projs[0];
    EXPECT_EQ( proj0.label, c.label );
    EXPECT_DOUBLE_EQ( proj0.label_probability, c.probability );

    // centroid
    EXPECT_EQ( (int)proj0.pose.pose.position.x, -9 );
    EXPECT_EQ( (int)proj0.pose.pose.position.y, 22 );
    EXPECT_DOUBLE_EQ( proj0.pose.pose.position.z, 0.5 );    // height is (currently) always 1

}