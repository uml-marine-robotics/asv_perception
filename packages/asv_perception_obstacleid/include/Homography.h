#ifndef OBSTACLEID_HOMOGRAPHY_H
#define OBSTACLEID_HOMOGRAPHY_H

#include <Eigen/Dense>
#include "defs.h"

namespace obstacle_id {

// represents a homography between 2d coordinates
class Homography {

    Eigen::Matrix3f _mat = {};

    public:
        // initialize from row-major float vector
        Homography( const std::vector<float>& data ) 
            : _mat( Eigen::Map<const Eigen::Matrix<float,3,3, Eigen::RowMajor>>( data.data() ) ) // a copy of the data is expected
        {}

        // transforms 2d point
        std::pair<float, float> operator()( float x, float y ) const {
            auto v = (this->_mat*Eigen::Vector3f(x, y, 1.f)).eval();
            v /= v[2];
            return std::make_pair( v[0], v[1] );
        }

};  // class
}   //ns
#endif