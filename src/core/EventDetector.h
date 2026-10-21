#pragma once

#include <string>
#include <vector>

#include "VehicleModel.h"

namespace driveforge {

enum class Severity { Info, Warning };

struct Event {
    double time = 0.0;  // simulated seconds
    Severity severity = Severity::Info;
    std::string message;
};

// Turns state transitions into log events. Edge-triggered with hysteresis so nothing is spammed.
class EventDetector {
public:
    static constexpr double kHardBrakeOn = 5.0;    // m/s^2 deceleration
    static constexpr double kHardBrakeOff = 2.0;
    static constexpr double kLateralOn = 4.0;      // m/s^2
    static constexpr double kLateralOff = 2.5;
    static constexpr double kStoppedSpeed = 0.05;  // m/s

    // Capture the current state as baseline without emitting events.
    void reset(const VehicleState& state, bool ignition, Gear gear);
    std::vector<Event> update(const VehicleState& state, bool ignition, Gear gear);

private:
    bool ignition_ = false;
    Gear gear_ = Gear::Drive;
    bool hardBraking_ = false;
    bool highLateral_ = false;
    bool moving_ = false;
};

}  // namespace driveforge
