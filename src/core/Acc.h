#pragma once

#include "Sensors.h"

namespace driveforge {

struct AccConfig {
    double standstillGap = 5.0;  // m
    double kSet = 0.5;           // 1/s, speed control
    double kSpeed = 0.8;         // 1/s, relative-speed term when following
    double kGap = 0.25;          // 1/s^2, gap error term
    double maxAccel = 2.0;       // m/s^2 comfort limits
    double maxDecel = 3.5;
    double laneHalfWidth = 1.8;  // m, lead-vehicle corridor
};

struct AccOutput {
    bool hasLead = false;
    int leadId = -1;
    double gap = 0.0;
    double desiredGap = 0.0;
    double accel = 0.0;  // commanded m/s^2
    double throttle = 0.0;
    double brake = 0.0;
};

// Adaptive cruise control: hold set speed, or follow the lead (vehicle/obstacle in lane) at standstillGap + timeGap * v.
// Returns pedal commands by inverting the longitudinal model of `params`.
AccOutput computeAcc(const AccConfig& config, double setSpeed, double timeGap, const VehicleState& state,
                     const VehicleParams& params, const SensorReadings& sensors, double maxBrakeDecel);

}  // namespace driveforge
