#pragma once

#include <limits>

#include "Sensors.h"

namespace driveforge {

struct AebConfig {
    double warnDecel = 2.5;      // m/s^2 required deceleration that raises a warning
    double brakeDecel = 4.0;     // m/s^2 required deceleration that triggers braking
    double gain = 1.3;           // brake demand = gain * required / available deceleration
    double minBrake = 0.4;
    double corridorMargin = 0.3; // m, added to half widths of ego and target
    double releaseDelay = 0.5;   // s without a threat before braking is released
    double minSpeed = 1.0;       // m/s, below this a new intervention is not started
    double ttcBrake = 1.7;       // s, brake whenever time-to-collision drops below this (stopping from 50 km/h takes ~2.3 s at 6 m/s^2)
    double ttcWarn = 2.5;        // s
    double closeRange = 2.0;     // m, brake if an in-path target is this close and getting closer
    double holdGap = 10.0;       // m, after stopping, hold the brake while a target is this close and the driver keeps accelerating
    double inhibitBase = 3.0;    // m, driver throttle is cut when an in-path target is closer than
    double inhibitHeadway = 0.5; //    inhibitBase + inhibitHeadway * speed (and not pulling away)
};

struct AebOutput {
    bool warning = false;
    bool braking = false;
    bool inhibitThrottle = false;
    double brake = 0.0;  // 0..1
    int targetId = -1;
    double ttc = std::numeric_limits<double>::infinity();
    double requiredDecel = 0.0;
};

// Automatic emergency braking: required deceleration v_close^2 / (2 gap) against the most critical in-path target.
// Pedestrians are predicted at constant lateral velocity until the ego car arrives.
class Aeb {
public:
    explicit Aeb(AebConfig config = {}) : config_(config) {}
    void reset();
    AebOutput update(const VehicleState& state, const VehicleParams& params, const SensorReadings& sensors,
                     double maxBrakeDecel, double driverThrottle, double dt);

private:
    AebConfig config_;
    bool latched_ = false;
    bool held_ = false;
    double noThreatTime_ = 0.0;
    double lastBrake_ = 0.0;
};

}  // namespace driveforge
