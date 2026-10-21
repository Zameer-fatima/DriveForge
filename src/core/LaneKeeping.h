#pragma once

#include <string>

#include "Road.h"
#include "Sensors.h"

namespace driveforge {

struct LkaConfig {
    double stanleyGain = 1.0;         // 1/s
    double softSpeed = 3.0;           // m/s, avoids singularity at low speed
    double comfortLateralAccel = 2.0; // m/s^2, caps the steering command
    double laneChangeDuration = 4.0;  // s, cosine lateral profile
    double completeOffsetTol = 0.25;  // m
    double completeHeadingTol = 0.05; // rad
    // Lane-change gap rules (edge to edge).
    double minFrontGap = 12.0, frontClosingTime = 2.0;
    double minRearGap = 10.0, rearClosingTime = 2.5;
};

struct LkaOutput {
    double steer = 0.0;       // -1..1
    double offset = 0.0;      // cross-track error to the (moving) reference, m
    bool laneChangeCompleted = false;
};

// Lane centring with the Stanley controller, plus a smooth lane-change manoeuvre.
class LaneKeeping {
public:
    explicit LaneKeeping(LkaConfig config = {}, Road road = {}) : config_(config), road_(road) {}

    // Target the lane the car is currently in; cancels any lane change.
    void engage(const VehicleState& state);
    void disengage();

    // dir: -1 left, +1 right. Returns false with a reason if refused.
    bool startLaneChange(int dir, const VehicleState& state, const SensorReadings& sensors, std::string* reason);

    LkaOutput update(const VehicleState& state, const VehicleParams& params, double dt);

    bool laneChanging() const { return changing_; }
    int laneChangeDirection() const { return changing_ ? changeDir_ : 0; }
    int targetLane() const { return changing_ ? toLane_ : targetLane_; }

private:
    LkaConfig config_;
    Road road_;
    int targetLane_ = 0;
    bool changing_ = false;
    int fromLane_ = 0, toLane_ = 0, changeDir_ = 0;
    double elapsed_ = 0.0;
};

}  // namespace driveforge
