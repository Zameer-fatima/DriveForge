#include "Aeb.h"

#include <algorithm>
#include <cmath>

namespace driveforge {

void Aeb::reset() {
    latched_ = false;
    held_ = false;
    noThreatTime_ = 0.0;
    lastBrake_ = 0.0;
}

AebOutput Aeb::update(const VehicleState& s, const VehicleParams& p, const SensorReadings& sensors,
                      double maxBrakeDecel, double driverThrottle, double dt) {
    AebOutput out;
    const bool forward = s.speed > 0.0;
    double ttcMin = std::numeric_limits<double>::infinity();
    double nearestInPathGap = std::numeric_limits<double>::infinity();

    for (const Detection& d : sensors.detections) {
        if (d.gap < 0.0) continue;
        const double corridor = p.width / 2.0 + d.halfWidth + config_.corridorMargin;
        if (std::abs(d.lateral) < corridor) nearestInPathGap = std::min(nearestInPathGap, d.gap);

        // Throttle inhibit: don't let the driver accelerate into a close in-path target.
        if (forward && std::abs(d.lateral) < corridor && d.closingSpeed > -0.5 &&
            d.gap < config_.inhibitBase + config_.inhibitHeadway * s.speed)
            out.inhibitThrottle = true;

        if (!forward || (s.speed < config_.minSpeed && !latched_) || d.closingSpeed <= 0.3) continue;

        // Threat assessment: constant closing speed; lateral position predicted until arrival.
        // The object is a threat if it is inside the corridor when our front arrives, when our rear has passed,
        // or it crosses the corridor in between (our body is 4.5 m long, so a walking pedestrian can step into the flank).
        const double ttc = d.gap / d.closingSpeed;
        const double ttcRear = (d.gap + p.length + 2.0 * d.halfLength) / d.closingSpeed;
        const double atFront = d.lateral + d.lateralSpeed * ttc;
        const double atRear = d.lateral + d.lateralSpeed * ttcRear;
        const bool inPath = std::abs(atFront) < corridor || std::abs(atRear) < corridor || atFront * atRear < 0.0;
        if (!inPath) continue;

        double required = d.closingSpeed * d.closingSpeed / (2.0 * std::max(d.gap, 0.1));
        if (ttc <= config_.ttcBrake || d.gap < config_.closeRange) required = std::max(required, config_.brakeDecel);
        if (required > out.requiredDecel) {
            out.requiredDecel = required;
            out.targetId = d.id;
        }
        ttcMin = std::min(ttcMin, ttc);
    }
    out.ttc = ttcMin;

    if (out.targetId >= 0) {
        noThreatTime_ = 0.0;
        if (out.requiredDecel >= config_.brakeDecel) latched_ = true;
    } else if (latched_) {
        noThreatTime_ += dt;
        if (noThreatTime_ >= config_.releaseDelay) latched_ = false;
    }

    // Stopped: keep holding the brake while the driver still pushes towards a close target.
    if (latched_ && s.speed < 0.05) {
        latched_ = false;
        held_ = nearestInPathGap < config_.holdGap;
    }
    if (held_ && (driverThrottle < 0.05 || nearestInPathGap >= config_.holdGap)) held_ = false;

    out.warning = (out.targetId >= 0 && (out.requiredDecel >= config_.warnDecel || ttcMin <= config_.ttcWarn)) || held_;
    if (latched_) {
        out.braking = true;
        out.warning = true;
        if (out.targetId >= 0)
            lastBrake_ = std::clamp(config_.gain * out.requiredDecel / maxBrakeDecel, config_.minBrake, 1.0);
        out.brake = std::max(lastBrake_, config_.minBrake);
    } else if (held_) {
        out.braking = true;
        out.brake = 1.0;
        lastBrake_ = 0.0;
    } else {
        lastBrake_ = 0.0;
    }
    if (held_ || latched_) out.inhibitThrottle = true;
    return out;
}

}  // namespace driveforge
