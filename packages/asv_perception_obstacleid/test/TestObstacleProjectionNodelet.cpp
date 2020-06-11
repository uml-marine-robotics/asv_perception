#include <iostream>
#include <asv_perception_common/ClassificationArray.h>
#include "../include/utils.h"
// #include "../include/obstacle_fusion.h"
#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>

namespace {
    using namespace obstacle_id;
}

TEST(TestObstacleProjectionNodelet, obstacle_fusion_fuse)
{   
    /*
    const int PX_VAL = 255;
    // create 100x100 img, with pixel values
    cv::Mat img(100,100, CV_8U,cv::Scalar(PX_VAL));

    asv_perception_common::ClassificationArray v = {};
    v.image_width = 100;
    v.image_height=100;
    
    auto c = asv_perception_common::Classification() ;
    c.roi.x_offset = 10;
    c.roi.y_offset = 10;
    c.roi.width = 20;
    c.roi.height = 30;
    v.classifications.emplace_back( c );

    auto fused = obstacle_fusion::fuse( img, v );
    ASSERT_EQ( fused.second.size(), 1 );

    auto img_f = fused.first;
    ASSERT_EQ( img_f.rows, img.rows );
    ASSERT_EQ( img_f.cols, img.cols );

    // get the roi of the classification from the resulting image, ensure it's been cleared
    auto cv_roi = utils::to_cv_rect( c.roi );
    ASSERT_EQ( cv::sum( img_f( cv_roi ) )[0], 0 );

    // offset by width, check again; should be area*orig_pixel_value
    cv_roi.x += cv_roi.width;
    ASSERT_EQ( cv::sum( img_f( cv_roi ) )[0], PX_VAL * cv_roi.width * cv_roi.height);
    */

}
