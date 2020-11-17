// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#ifndef OBSTACLEID_HOMOGRAPHY_H
#define OBSTACLEID_HOMOGRAPHY_H

#include <Eigen/Dense>
#include "defs.h"

namespace obstacle_id {
namespace detail {

// represents a homography between 2d coordinates
class Homography {

    // store/calculate as double to prevent overflows when transform x,y approach float max
    Eigen::Matrix3d _mat = {};

    public:

        // initialize from row-major float ptr
        Homography( const float* const data ) 
            : _mat( Eigen::Map<const Eigen::Matrix<float,3,3, Eigen::RowMajor>>( data ).cast<double>() ) // data should be copied to _mat
        {}

        Homography( const Eigen::Matrix3d& mat )
            : _mat(mat)
        {}

        // homography matrix coefficient access
        float at( const int row, const int column ) const { return (float)this->_mat( row, column ); }

        // transforms 2d point
        std::pair<float, float> operator()( const float x, const float y ) const {
            Eigen::Vector3d v = this->_mat * Eigen::Vector3d(x, y, 1.);
            return std::make_pair( (float)( v[0] / v[2] ), (float)( v[1] / v[2] ) );
        }

        // returns the inverse of this Homography
        Homography inverse() const {
            return { this->_mat.inverse() };
        }

};  // class
}}   //ns
#endif