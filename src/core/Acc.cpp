#include "Acc.h"

#include <algorithm>
#include <cmath>

namespace driveforge {

AccOutput computeAcc(const AccConfig& c, double setSpeed, double timeGap, const VehicleState& s,
                     const VehicleParams& p, const SensorReadings& sensors, double maxBrakeDecel) {
    AccOutput out;
    const double v = std::max(s.speed, 0.0);

    const Detection* lead = nullptr;
    for (const Detection& d : sensors.detections) {
        if (d.type == ObjectType::Pedestrian || d.gap < 0.0) continue;
        if (std::abs(d.lateral) < c.laneHalfWidth) {
            lead = &d;
            break;
        }
    }

    double accel = c.kSet * (setSpeed - v);
    if (lead) {
        out.hasLead = true;
        out.leadId = lead->id;
        out.gap = lead->gap;
        out.desiredGap = c.standstillGap + timeGap * v;
        const double leadSpeed = v - lead->closingSpeed;
        const double follow = c.kSpeed * (leadSpeed - v) + c.kGap * (lead->gap - out.desiredGap);
        accel = std::min(accel, follow);
    }
    accel = std::clamp(accel, -c.maxDecel, c.maxAccel);
    out.accel = accel;

    // Invert the longitudinal model: accel = drive - resistance - brake * maxBrakeDecel.
    const double resist = p.rollingResistance + p.dragCoefficient * v * v;
    const double drive = accel + resist;
    if (drive >= 0.0) {
        const double available = p.maxAccel * std::max(0.05, 1.0 - v / p.maxSpeedForward);
        out.throttle = std::clamp(drive / available, 0.0, 1.0);
    } else {
        out.brake = std::clamp(-drive / maxBrakeDecel, 0.0, 1.0);
    }
    return out;
}

}  // namespace driveforge
