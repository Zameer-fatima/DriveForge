#include "VehicleModel.h"

#include <algorithm>
#include <cmath>

namespace driveforge {

namespace {

constexpr double kGravity = 9.81;
constexpr double kGearChangeSpeed = 0.1;  // m/s

double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

// Move value toward target by at most rate*dt.
double approach(double value, double target, double riseRate, double fallRate, double dt) {
    if (target > value) return std::min(target, value + riseRate * dt);
    return std::max(target, value - fallRate * dt);
}

}  // namespace

VehicleModel::VehicleModel(VehicleParams params) : params_(params) {}

void VehicleModel::reset(double x, double y, double heading, double speed) {
    state_ = VehicleState{};
    state_.x = x;
    state_.y = y;
    state_.heading = heading;
    state_.speed = speed;
    gear_ = speed < 0.0 ? Gear::Reverse : Gear::Drive;
    input_ = ControlInput{};
    input_.gear = gear_;
}

void VehicleModel::setInput(const ControlInput& input) {
    input_ = input;
    input_.throttle = clamp(input.throttle, 0.0, 1.0);
    input_.brake = clamp(input.brake, 0.0, 1.0);
    input_.steer = clamp(input.steer, -1.0, 1.0);
}

double VehicleModel::maxBrakeDeceleration() const {
    return std::min(params_.maxBrake, params_.tyreFriction * kGravity);
}

double VehicleModel::brakingDistance(double speed) const {
    return speed * speed / (2.0 * maxBrakeDeceleration());
}

double VehicleModel::steerLimit(double speed) const {
    const double v2 = speed * speed;
    if (v2 < 1e-6) return params_.maxSteerAngle;
    const double byLateral = std::atan(params_.maxLateralAccel * params_.wheelbase / v2);
    return std::min(params_.maxSteerAngle, byLateral);
}

void VehicleModel::step(double dt) {
    if (!(dt > 0.0)) return;
    const VehicleParams& p = params_;
    VehicleState& s = state_;

    // Gear selection only when (almost) stationary.
    if (input_.gear != gear_ && std::abs(s.speed) < kGearChangeSpeed) gear_ = input_.gear;

    // Pedal actuators.
    const double throttleCmd = input_.ignition ? input_.throttle : 0.0;
    s.throttle = approach(s.throttle, throttleCmd, p.pedalRiseRate, p.pedalFallRate, dt);
    s.brake = approach(s.brake, input_.brake, p.pedalRiseRate, p.pedalFallRate, dt);

    // Longitudinal dynamics.
    double driveAccel;
    if (gear_ == Gear::Drive) {
        driveAccel = s.throttle * p.maxAccel * std::max(0.0, 1.0 - std::max(0.0, s.speed) / p.maxSpeedForward);
    } else {
        driveAccel = -s.throttle * p.maxAccel * p.reverseAccelFactor *
                     std::max(0.0, 1.0 - std::max(0.0, -s.speed) / p.maxSpeedReverse);
    }
    const double previousSpeed = s.speed;
    const double driven = s.speed + driveAccel * dt;
    const double mag = std::abs(driven);
    const double resist = p.rollingResistance + p.dragCoefficient * mag * mag;
    const double slowdown = (resist + s.brake * maxBrakeDeceleration()) * dt;
    const double newMag = std::max(0.0, mag - slowdown);
    s.speed = std::copysign(newMag, driven);
    s.acceleration = (s.speed - previousSpeed) / dt;

    // Steering actuator, limited by lateral acceleration at the current speed.
    const double limit = steerLimit(s.speed);
    const double target = clamp(input_.steer * p.maxSteerAngle, -limit, limit);
    s.steerAngle = approach(s.steerAngle, target, p.steerRate, p.steerRate, dt);
    s.steerAngle = clamp(s.steerAngle, -limit, limit);

    // Kinematic bicycle integration (midpoint heading).
    const double yawRate = s.speed / p.wheelbase * std::tan(s.steerAngle);
    const double midHeading = s.heading + 0.5 * yawRate * dt;
    s.x += s.speed * std::sin(midHeading) * dt;
    s.y += s.speed * std::cos(midHeading) * dt;
    s.heading += yawRate * dt;
    s.lateralAcceleration = s.speed * yawRate;
    s.travel += s.speed * dt;
    s.time += dt;
}

}  // namespace driveforge
