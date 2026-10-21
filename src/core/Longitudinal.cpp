#include "Longitudinal.h"

#include <algorithm>
#include <cmath>

namespace driveforge {

Pedals pedalsForAccel(double accel, double speed, Gear gear, const VehicleParams& p, double maxBrakeDecel) {
    const double mag = std::abs(speed);
    const double resist = p.rollingResistance + p.dragCoefficient * mag * mag;
    const double resistAccel = mag > 1e-3 ? -std::copysign(resist, speed) : 0.0;
    const double need = accel - resistAccel;  // what drive or brake must supply

    Pedals out;
    if (gear == Gear::Drive) {
        if (need >= 0.0) {
            const double available = p.maxAccel * std::max(0.05, 1.0 - std::max(speed, 0.0) / p.maxSpeedForward);
            out.throttle = std::clamp(need / available, 0.0, 1.0);
        } else {
            out.brake = std::clamp(-need / maxBrakeDecel, 0.0, 1.0);
        }
    } else {
        if (need <= 0.0) {
            const double available = p.maxAccel * p.reverseAccelFactor *
                                     std::max(0.05, 1.0 - std::max(-speed, 0.0) / p.maxSpeedReverse);
            out.throttle = std::clamp(-need / available, 0.0, 1.0);
        } else {
            out.brake = std::clamp(need / maxBrakeDecel, 0.0, 1.0);
        }
    }
    return out;
}

Pedals pedalsForSpeed(double target, double speed, Gear gear, const VehicleParams& p, double maxBrakeDecel) {
    if (std::abs(target) < 1e-3 && std::abs(speed) < 0.05) return {0.0, 1.0};
    const double accel = std::clamp(5.0 * (target - speed), -2.5, 1.5);
    return pedalsForAccel(accel, speed, gear, p, maxBrakeDecel);
}

}  // namespace driveforge
