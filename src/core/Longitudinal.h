#pragma once

#include "VehicleModel.h"

namespace driveforge {

struct Pedals {
    double throttle = 0.0;
    double brake = 0.0;
};

// Pedal commands that produce `accel` (signed m/s^2 along +y) by inverting the longitudinal model of `params`.
// `speed` is signed; `gear` is the gear actually engaged.
Pedals pedalsForAccel(double accel, double speed, Gear gear, const VehicleParams& params, double maxBrakeDecel);

// Proportional speed controller towards a signed target speed. A (near) zero target at standstill holds the brake.
Pedals pedalsForSpeed(double targetSpeed, double speed, Gear gear, const VehicleParams& params, double maxBrakeDecel);

}  // namespace driveforge
