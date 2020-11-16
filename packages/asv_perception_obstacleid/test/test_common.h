// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#include <vector>
#include "../include/detail/Homography.h"

namespace {

    using namespace obstacle_id;
    using namespace obstacle_id::detail;
    
    // camera image (1280x1024) to world matrix
    static const std::vector<float> HOMOGRAPHY_RGB_TO_WORLD = { 
        0.47386, -0.113292, -291.825,
        0.053203, -0.387605, 674.903,
        0.0163188, 0.311054, -103.878
    };
}

inline Homography getRGBtoWorldHomography() {
    return Homography( ::HOMOGRAPHY_RGB_TO_WORLD.data() );
}