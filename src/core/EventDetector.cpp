#include "EventDetector.h"

#include <cmath>
#include <cstdio>

namespace driveforge {

namespace {
std::string format(const char* fmt, double a) {
    char buf[96];
    std::snprintf(buf, sizeof buf, fmt, a);
    return buf;
}
}  // namespace

void EventDetector::reset(const VehicleState& state, bool ignition, Gear gear) {
    ignition_ = ignition;
    gear_ = gear;
    hardBraking_ = false;
    highLateral_ = false;
    moving_ = std::abs(state.speed) >= kStoppedSpeed;
}

std::vector<Event> EventDetector::update(const VehicleState& s, bool ignition, Gear gear) {
    std::vector<Event> out;
    auto add = [&](Severity sev, std::string msg) { out.push_back({s.time, sev, std::move(msg)}); };

    if (ignition != ignition_) {
        ignition_ = ignition;
        add(Severity::Info, ignition ? "Ignition ON" : "Ignition OFF");
    }
    if (gear != gear_) {
        gear_ = gear;
        add(Severity::Info, gear == Gear::Reverse ? "Gear changed to Reverse" : "Gear changed to Drive");
    }

    const bool movingNow = std::abs(s.speed) >= kStoppedSpeed;
    if (movingNow != moving_) {
        moving_ = movingNow;
        add(Severity::Info, movingNow ? "Vehicle moving" : "Vehicle stopped");
    }

    const double decel = std::abs(s.speed) > 1.0 ? -s.acceleration * (s.speed > 0 ? 1.0 : -1.0) : 0.0;
    if (!hardBraking_ && decel > kHardBrakeOn) {
        hardBraking_ = true;
        add(Severity::Warning, format("Hard braking (%.1f m/s^2)", decel));
    } else if (hardBraking_ && decel < kHardBrakeOff) {
        hardBraking_ = false;
    }

    const double lat = std::abs(s.lateralAcceleration);
    if (!highLateral_ && lat > kLateralOn) {
        highLateral_ = true;
        add(Severity::Warning, format("High lateral acceleration (%.1f m/s^2)", lat));
    } else if (highLateral_ && lat < kLateralOff) {
        highLateral_ = false;
    }
    return out;
}

}  // namespace driveforge
