#pragma once

#include <algorithm>
#include <cmath>

namespace driveforge {

// Straight multi-lane road along +y. Lane 0 is the leftmost lane (negative x). Scene3D draws the same geometry.
struct Road {
    int laneCount = 3;
    double laneWidth = 3.6;

    double laneCenter(int lane) const { return (lane - (laneCount - 1) / 2.0) * laneWidth; }
    double halfWidth() const { return laneCount * laneWidth / 2.0; }
    int laneIndexAt(double x) const {
        const int i = static_cast<int>(std::lround(x / laneWidth + (laneCount - 1) / 2.0));
        return std::min(std::max(i, 0), laneCount - 1);
    }
    // Lane markings are detectable by the camera while the car is on or just beside the road.
    bool laneVisible(double x) const { return std::abs(x) <= halfWidth() + 1.0; }
};

}  // namespace driveforge
