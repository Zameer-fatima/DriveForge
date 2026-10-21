#include "Telemetry.h"

namespace driveforge {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double channelValue(const TelemetrySample& s, TelemetryChannel channel) {
    switch (channel) {
    case TelemetryChannel::Speed: return s.speedKmh;
    case TelemetryChannel::Acceleration: return s.acceleration;
    case TelemetryChannel::LateralAcceleration: return s.lateralAcceleration;
    case TelemetryChannel::Steering: return s.steeringDeg;
    case TelemetryChannel::Throttle: return s.throttle;
    case TelemetryChannel::Brake: return s.brake;
    case TelemetryChannel::FrontGap: return s.frontGap;
    }
    return 0.0;
}

Telemetry::Telemetry(double period, std::size_t capacity) : period_(period > 0.0 ? period : 0.1), samples_(capacity) {}

bool Telemetry::update(const VehicleState& state, double frontGap) {
    if (state.time + 1e-9 < nextTime_) return false;
    TelemetrySample s;
    s.time = state.time;
    s.speedKmh = state.speed * 3.6;
    s.acceleration = state.acceleration;
    s.lateralAcceleration = state.lateralAcceleration;
    s.steeringDeg = state.steerAngle * 180.0 / kPi;
    s.throttle = state.throttle;
    s.brake = state.brake;
    s.frontGap = frontGap;
    samples_.push(s);
    nextTime_ = state.time + period_;
    return true;
}

void Telemetry::clear() {
    samples_.clear();
    nextTime_ = 0.0;
}

}  // namespace driveforge
