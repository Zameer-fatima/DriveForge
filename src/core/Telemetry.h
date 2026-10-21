#pragma once

#include "RingBuffer.h"
#include "VehicleModel.h"

namespace driveforge {

// One recorded sample of simulated vehicle state. Units: s, km/h, m/s^2, deg, 0..1.
struct TelemetrySample {
    double time = 0.0;
    double speedKmh = 0.0;
    double acceleration = 0.0;
    double lateralAcceleration = 0.0;
    double steeringDeg = 0.0;
    double throttle = 0.0;
    double brake = 0.0;
    double frontGap = 150.0;  // m, nearest object ahead in path (sensor range if none)
};

enum class TelemetryChannel { Speed = 0, Acceleration, LateralAcceleration, Steering, Throttle, Brake, FrontGap };

double channelValue(const TelemetrySample& s, TelemetryChannel channel);

// Samples vehicle state at a fixed simulated-time period into a ring buffer.
class Telemetry {
public:
    explicit Telemetry(double period = 0.1, std::size_t capacity = 600);

    // Call every simulation step. Returns true if a new sample was recorded.
    bool update(const VehicleState& state, double frontGap = 150.0);
    void clear();

    const RingBuffer<TelemetrySample>& samples() const { return samples_; }
    double period() const { return period_; }

private:
    double period_;
    double nextTime_ = 0.0;
    RingBuffer<TelemetrySample> samples_;
};

}  // namespace driveforge
