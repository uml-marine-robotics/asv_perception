#include <vector>
#include "../include/detail/Homography.h"

namespace {

    using namespace obstacle_id;
    using namespace obstacle_id::detail;
    
    // image (1280x1024) to world matrix (radar img:  1024x1024, 220m real-world diameter)
    static const std::vector<float> HOMOGRAPHY_RGB_TO_WORLD = { 
        0.47386, -0.113292, -291.825,
        0.053203, -0.387605, 674.903,
        0.0163188, 0.311054, -103.878
    };

    // image (1280x1024) to radar matrix (radar img:  1024x1024, 220m real-world diameter)
    static const std::vector<float> HOMOGRAPHY_RGB_TO_RADAR = { 
        2.268932, 34.102644, -11718.441166,
        1.741869, 34.603541, -12101.519025,
        0.003506, 0.066828, -22.317610
    };
}

inline Homography getRGBtoWorldHomography() {
    return Homography( ::HOMOGRAPHY_RGB_TO_WORLD.data() );
}

inline Homography getRGBtoRadarHomography() {
    return Homography( ::HOMOGRAPHY_RGB_TO_RADAR.data() );
}