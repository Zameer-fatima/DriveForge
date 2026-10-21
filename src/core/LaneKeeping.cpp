#include "LaneKeeping.h"

#include <algorithm>
#include <cmath>

namespace driveforge {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

void LaneKeeping::engage(const VehicleState& s) {
    targetLane_ = road_.laneIndexAt(s.x);
    changing_ = false;
    elapsed_ = 0.0;
}

void LaneKeeping::disengage() { changing_ = false; }

bool LaneKeeping::canChangeLane(int dir, const VehicleState& s, const SensorReadings& sensors, std::string* reason) const {
    auto refuse = [&](const char* why) {
        if (reason) *reason = why;
        return false;
    };
    if (changing_) return refuse("lane change already in progress");
    const int target = targetLane_ + (dir < 0 ? -1 : 1);
    if (target < 0 || target >= road_.laneCount) return refuse("no lane in that direction");
    if (s.speed < 2.0) return refuse("speed too low for a lane change");

    const double center = road_.laneCenter(target);
    for (const Detection& d : sensors.detections) {
        if (std::abs(d.worldX - center) >= road_.laneWidth / 2.0 + d.halfWidth) continue;  // not in target lane
        if (d.gap >= 0.0) {
            if (d.gap < config_.minFrontGap + config_.frontClosingTime * std::max(0.0, d.closingSpeed))
                return refuse("target lane occupied ahead");
        } else if (-d.gap < config_.minRearGap + config_.rearClosingTime * std::max(0.0, -d.closingSpeed)) {
            return refuse("target lane occupied behind");
        }
    }
    return true;
}

bool LaneKeeping::startLaneChange(int dir, const VehicleState& s, const SensorReadings& sensors, std::string* reason) {
    if (!canChangeLane(dir, s, sensors, reason)) return false;
    fromLane_ = targetLane_;
    toLane_ = targetLane_ + (dir < 0 ? -1 : 1);
    changeDir_ = dir < 0 ? -1 : 1;
    changing_ = true;
    elapsed_ = 0.0;
    return true;
}

LkaOutput LaneKeeping::update(const VehicleState& s, const VehicleParams& p, double dt) {
    LkaOutput out;
    const double v = std::max(s.speed, 0.0);

    double xRef = road_.laneCenter(targetLane_);
    double xDot = 0.0;
    double progress = 0.0;
    if (changing_) {
        elapsed_ += dt;
        progress = std::min(elapsed_ / config_.laneChangeDuration, 1.0);
        const double x0 = road_.laneCenter(fromLane_), x1 = road_.laneCenter(toLane_);
        xRef = x0 + (x1 - x0) * (1.0 - std::cos(kPi * progress)) / 2.0;
        if (progress < 1.0) xDot = (x1 - x0) * kPi / (2.0 * config_.laneChangeDuration) * std::sin(kPi * progress);
    }

    const double e = s.x - xRef;
    const double psiRef = std::atan2(xDot, std::max(v, 0.1));
    const double psiErr = s.heading - psiRef;
    double delta = -(psiErr + std::atan2(config_.stanleyGain * e, v + config_.softSpeed));
    // Comfort limit: v^2 tan(delta) / L <= a_comfort.
    const double comfort = std::atan(config_.comfortLateralAccel * p.wheelbase / std::max(v * v, 1.0));
    delta = std::clamp(delta, -comfort, comfort);
    out.steer = std::clamp(delta / p.maxSteerAngle, -1.0, 1.0);
    out.offset = e;

    if (changing_ && progress >= 1.0 && std::abs(e) < config_.completeOffsetTol &&
        std::abs(psiErr) < config_.completeHeadingTol) {
        changing_ = false;
        targetLane_ = toLane_;
        out.laneChangeCompleted = true;
    }
    return out;
}

}  // namespace driveforge
