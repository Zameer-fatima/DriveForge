#pragma once

namespace driveforge {

enum class Gear { Drive, Reverse };

// All values SI (m, s, rad). See docs/DESIGN.md for the model equations.
struct VehicleParams {
    double wheelbase = 2.7;
    double length = 4.5;
    double width = 1.85;
    double wheelRadius = 0.33;

    double maxSteerAngle = 0.61;      // rad (~35 deg) at the wheels
    double steerRate = 0.9;           // rad/s
    double maxLateralAccel = 7.0;     // m/s^2, limits steering at speed

    double maxAccel = 3.5;            // m/s^2 at zero speed, full throttle
    double maxSpeedForward = 50.0;    // m/s
    double maxSpeedReverse = 5.0;     // m/s
    double reverseAccelFactor = 0.6;
    double maxBrake = 8.0;            // m/s^2, actuator limit
    double tyreFriction = 0.9;        // mu, dry asphalt
    double rollingResistance = 0.15;  // m/s^2
    double dragCoefficient = 0.0003;  // 1/m, a = c * v^2

    double pedalRiseRate = 4.0;       // 1/s
    double pedalFallRate = 6.0;       // 1/s
};

struct ControlInput {
    double throttle = 0.0;  // 0..1
    double brake = 0.0;     // 0..1
    double steer = 0.0;     // -1 (left) .. 1 (right)
    Gear gear = Gear::Drive;
    bool ignition = false;
};

struct VehicleState {
    double x = 0.0;                    // m, right
    double y = 0.0;                    // m, forward along road
    double heading = 0.0;              // rad, 0 = +y, positive = clockwise
    double speed = 0.0;                // m/s, signed (negative = reversing)
    double acceleration = 0.0;         // m/s^2 longitudinal, last step
    double lateralAcceleration = 0.0;  // m/s^2
    double steerAngle = 0.0;           // rad, actual, positive = right
    double throttle = 0.0;             // actual actuator 0..1
    double brake = 0.0;                // actual actuator 0..1
    double travel = 0.0;               // m, signed distance rolled (wheel spin)
    double time = 0.0;                 // s
};

class VehicleModel {
public:
    explicit VehicleModel(VehicleParams params = {});

    void reset(double x = 0.0, double y = 0.0, double heading = 0.0, double speed = 0.0);
    void setInput(const ControlInput& input);
    void step(double dt);
    // Used by the collision handler: the car stops dead on impact.
    void forceSpeed(double speed) { state_.speed = speed; }

    const VehicleState& state() const { return state_; }
    const VehicleParams& params() const { return params_; }
    const ControlInput& input() const { return input_; }
    Gear gear() const { return gear_; }
    bool ignition() const { return input_.ignition; }

    // Deceleration achievable at full brake (limited by actuator and tyre friction).
    double maxBrakeDeceleration() const;
    // v^2 / (2 a). Ignores drag and pedal ramp-up.
    double brakingDistance(double speed) const;
    // Largest |steer angle| allowed at this speed (lateral-acceleration limit).
    double steerLimit(double speed) const;

private:
    VehicleParams params_;
    ControlInput input_;
    VehicleState state_;
    Gear gear_ = Gear::Drive;
};

}  // namespace driveforge
