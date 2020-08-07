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

TEST( TEST_CASE_NAME, roi_adjustment_larger )
{   
    cv::Mat img( 1024, 1280, CV_8U,cv::Scalar(0));
    
    // create a roi
    sensor_msgs::RegionOfInterest roi = {};
    roi.x_offset = 150;
    roi.width = 100;
    roi.y_offset = 400;
    roi.height = 50;

    // create larger rectangle in image
    auto rect_larger = utils::to_cv_rect( roi, img );    // get opencv rect based on roi
    rect_larger.y -= 10;
    rect_larger.height += 20;

    img( rect_larger ) = 255;    // set roi to obstacle class in obstacle map

    // cv::imwrite("/catkin_ws/src/larger.bmp", img );

    // do adjustment of roi
    const auto adjusted_roi = classified_obstacle_projection::impl::adjust_vertical_roi_to_image( roi, img );

    // check
    EXPECT_EQ( adjusted_roi.x_offset, roi.x_offset );
    EXPECT_EQ( adjusted_roi.width, roi.width );
    EXPECT_EQ( adjusted_roi.y_offset, roi.y_offset - 10 );
    EXPECT_EQ( adjusted_roi.height, roi.height + 20 );
}

TEST( TEST_CASE_NAME, roi_adjustment_smaller )
{   
    cv::Mat img( 1024, 1280, CV_8U,cv::Scalar(0));
    
    // create a roi
    sensor_msgs::RegionOfInterest roi = {};
    roi.x_offset = 150;
    roi.width = 100;
    roi.y_offset = 400;
    roi.height = 50;

    // create smaller rectangle in image
    auto rect_smaller = utils::to_cv_rect( roi, img );    // get opencv rect based on roi
    rect_smaller.y += 10;
    rect_smaller.height -= 20;

    EXPECT_EQ( rect_smaller.y, 410 );
    EXPECT_EQ( rect_smaller.height, 30 );

    img( rect_smaller ) = 255;    // set roi to obstacle class in obstacle map

    //cv::imwrite("/catkin_ws/src/smaller.bmp", img );

    // do adjustment of roi
    const auto adjusted_roi = classified_obstacle_projection::impl::adjust_vertical_roi_to_image( roi, img );

    // check
    EXPECT_EQ( adjusted_roi.x_offset, roi.x_offset );
    EXPECT_EQ( adjusted_roi.width, roi.width );
    EXPECT_EQ( adjusted_roi.y_offset, roi.y_offset + 10 );
    EXPECT_EQ( adjusted_roi.height, roi.height - 20 );
}

TEST( TEST_CASE_NAME, roi_adjustment_img_limits )
{   
    // image is entirely obstacle class, check roi expansion

    cv::Mat img( 1024, 1280, CV_8U,cv::Scalar(255));
    
    // create a roi
    sensor_msgs::RegionOfInterest roi = {};
    roi.x_offset = 150;
    roi.width = 100;
    roi.y_offset = 400;
    roi.height = 50;

    // do adjustment of roi
    const auto adjusted_roi = classified_obstacle_projection::impl::adjust_vertical_roi_to_image( roi, img );

    // check
    EXPECT_EQ( adjusted_roi.x_offset, roi.x_offset );
    EXPECT_EQ( adjusted_roi.width, roi.width );
    EXPECT_EQ( adjusted_roi.y_offset, 0 );
    EXPECT_EQ( adjusted_roi.height, img.rows );
}

TEST( TEST_CASE_NAME, roi_adjustment_no_op )
{   
    // image is entirely blank, check roi expansion is no-op

    cv::Mat img( 1024, 1280, CV_8U,cv::Scalar(0));
    
    // create a roi
    sensor_msgs::RegionOfInterest roi = {};
    roi.x_offset = 150;
    roi.width = 100;
    roi.y_offset = 400;
    roi.height = 50;

    // do adjustment of roi
    const auto adjusted_roi = classified_obstacle_projection::impl::adjust_vertical_roi_to_image( roi, img );

    // check
    EXPECT_EQ( adjusted_roi.x_offset, roi.x_offset );
    EXPECT_EQ( adjusted_roi.width, roi.width );
    EXPECT_EQ( adjusted_roi.y_offset, roi.y_offset );
    EXPECT_EQ( adjusted_roi.height, roi.height );
}

TEST( TEST_CASE_NAME, classified_obstacle_2d_intersection_area )
{   
    auto c1 = asv_perception_common::Classification() ;
    c1.roi.x_offset = 150;
    c1.roi.y_offset = 400;
    c1.roi.height = 50;
    c1.roi.width = 250;
    auto c1_cls = ClassifiedObstacle2d(c1);

    // c2 overlaps c1
    auto c2 = asv_perception_common::Classification() ;
    c2.roi.x_offset = 100;
    c2.roi.y_offset = 410;
    c2.roi.height = 50;
    c2.roi.width = 250;
    auto c2_cls = ClassifiedObstacle2d(c2);

    // c3 does not overlap c1
    auto c3 = asv_perception_common::Classification() ;
    c3.roi.x_offset = 0;
    c3.roi.y_offset = 410;
    c3.roi.height = 50;
    c3.roi.width = 50;
    auto c3_cls = ClassifiedObstacle2d(c3);

    EXPECT_GT( c1_cls.roi_intersection_area(c2_cls), 0.f );
    EXPECT_FLOAT_EQ( c1_cls.roi_intersection_area(c3_cls), 0.f );
}

TEST( TEST_CASE_NAME, roi_limited_adjustment_smaller )
{   
    cv::Mat img1( 1024, 1280, CV_8U,cv::Scalar(0));

    std::vector<std::shared_ptr<ClassifiedObstacle2d>> obs = {};

    auto c1 = asv_perception_common::Classification() ;
    c1.roi.x_offset = 150;
    c1.roi.y_offset = 400;
    c1.roi.height = 50;
    c1.roi.width = 250;
    auto roi = c1.roi;
    
    obs.emplace_back( std::make_shared<ClassifiedObstacle2d>(c1) );

    auto obs_ptr = obs.front();

    // create rectangle in image1
    auto rect = utils::to_cv_rect( obs_ptr->cls.roi, img1 );    // get opencv rect based on roi
    rect.y += 10;
    rect.height += 30;      // 60% increase
    img1( rect ) = 255;    // set roi to obstacle class in obstacle map

    // do adjustment of roi, expect success
    EXPECT_TRUE( classified_obstacle_projection::impl::adjust_roi( obs.front(), obs, img1, 0.7f, 0.f ) );

    // check changes were made to obstacle
    EXPECT_EQ( obs_ptr->cls.roi.x_offset, roi.x_offset );
    EXPECT_EQ( obs_ptr->cls.roi.width, roi.width );
    EXPECT_EQ( obs_ptr->cls.roi.y_offset, rect.y );
    EXPECT_EQ( obs_ptr->cls.roi.height, rect.height );
}

TEST( TEST_CASE_NAME, roi_adjustment_limited_smaller )
{   
    cv::Mat img1( 1024, 1280, CV_8U,cv::Scalar(0) );

    std::vector<std::shared_ptr<ClassifiedObstacle2d>> obs = {};

    auto c1 = asv_perception_common::Classification() ;
    c1.roi.x_offset = 150;
    c1.roi.y_offset = 400;
    c1.roi.height = 50;
    c1.roi.width = 250;
    auto roi = c1.roi;
    
    obs.emplace_back( std::make_shared<ClassifiedObstacle2d>(c1) );

    auto obs_ptr = obs.front();

    // create smaller rectangle in image1
    auto rect_smaller = utils::to_cv_rect( obs_ptr->cls.roi, img1 );    // get opencv rect based on roi
    rect_smaller.y += 10;
    rect_smaller.height -= 30;      // 60% reduction
    img1( rect_smaller ) = 255;    // set roi to obstacle class in obstacle map

    // do adjustment of roi, should fail
    EXPECT_FALSE( classified_obstacle_projection::impl::adjust_roi( obs.front(), obs, img1, 0., 0.5 ) );

}

TEST( TEST_CASE_NAME, roi_adjustment_limited_larger )
{   
    cv::Mat img1( 1024, 1280, CV_8U,cv::Scalar(0) );

    std::vector<std::shared_ptr<ClassifiedObstacle2d>> obs = {};

    auto c1 = asv_perception_common::Classification() ;
    c1.roi.x_offset = 150;
    c1.roi.y_offset = 400;
    c1.roi.height = 50;
    c1.roi.width = 250;
    auto roi = c1.roi;
    
    obs.emplace_back( std::make_shared<ClassifiedObstacle2d>(c1) );

    auto obs_ptr = obs.front();

    // create smaller rectangle in image1
    auto rect_smaller = utils::to_cv_rect( obs_ptr->cls.roi, img1 );    // get opencv rect based on roi
    rect_smaller.y += 10;
    rect_smaller.height += 30;      // 60% increase
    img1( rect_smaller ) = 255;    // set roi to obstacle class in obstacle map

    // do adjustment of roi
    EXPECT_FALSE( classified_obstacle_projection::impl::adjust_roi( obs.front(), obs, img1, 0.5f, 0.f ) );

}