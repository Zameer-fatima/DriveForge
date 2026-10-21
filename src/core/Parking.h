#pragma once

#include <string>
#include <vector>

#include "EventDetector.h"
#include "Road.h"
#include "Sensors.h"
#include "VehicleModel.h"

namespace driveforge {

enum class ParkingPhase {
    Off,
    Searching,   // looking for a free bay with a feasible plan
    Ready,       // a bay and plan are available; waiting for the driver to start
    Align,       // drive along the road to the start point of the reverse arc
    PreSteer,    // stationary: turn the wheels to the arc angle
    Arc,         // reverse along a constant-radius arc until the car is perpendicular to the road
    Straighten,  // stationary: straighten the wheels
    Straight,    // reverse straight into the bay
    Settle,
    Parked,
};

struct ParkingConfig {
    double steerUsage = 0.9;        // fraction of max steering angle allowed for the tightest arc
    double radiusStep = 0.25;       // m, planner search step
    double clearance = 0.2;         // m, required margin to other objects along the planned path
    double abortClearance = 0.05;   // m, margin below which an obstacle ahead aborts parking
    double lookahead = 2.5;         // m of planned path checked ahead of the car
    double alignSpeed = 4.0;        // m/s
    double creepSpeed = 0.12;       // m/s, final approach so targets are always reached
    double slotTolerance = 0.1;     // m, allowed overhang of the car outside the bay lines
    double arcSpeed = 1.2;
    double straightSpeed = 1.5;
    double comfortDecel = 1.5;      // m/s^2 used for the approach speed profiles
    double maxStartSpeed = 30.0;
    double headingTol = 0.12;       // rad, heading error allowed to plan/start
    double alignTol = 0.15;         // m
    double finalHeadingTol = 0.10;  // rad, accepted at the end
    double maxDuration = 240.0;     // s
    double planPeriod = 0.2;        // s between re-plans while searching
};

struct ParkingPose {
    double x = 0.0, y = 0.0, heading = 0.0;
};

// Geometry for a perpendicular reverse-in bay on the right: straight along the road to (x0, y0), then a reverse arc of
// radius R (steering right) until the car points across the road, then straight back by S to the bay centre (xf, yf).
struct ParkingPlan {
    bool valid = false;
    int slotId = -1;
    double x0 = 0.0, y0 = 0.0;  // start of the arc (car pose with heading 0)
    double radius = 0.0;        // m
    double straight = 0.0;      // m, final straight reverse
    double steerAngle = 0.0;    // rad, wheels during the arc
    double xf = 0.0, yf = 0.0;  // final car centre
    double clearance = 0.0;     // m, margin to other objects along the path
    std::vector<ParkingPose> path;  // arc + final straight, ~0.25 m spacing
    std::string reason;         // why no plan (when !valid)
};

struct ParkingCommand {
    bool active = false;  // true while the assist is driving the car
    double throttle = 0.0;
    double brake = 0.0;
    double steer = 0.0;   // -1..1
    Gear gear = Gear::Drive;
};

class ParkingAssist {
public:
    explicit ParkingAssist(ParkingConfig config = {}, Road road = {}) : config_(config), road_(road) {}

    void reset();
    void setTime(double time) { time_ = time; }  // keeps event timestamps current between updates
    void setEnabled(bool enabled);
    bool enabled() const { return phase_ != ParkingPhase::Off; }
    bool active() const;  // driving the car
    // Begin parking into the current plan. Returns false (with a reason) if not possible.
    bool start(const VehicleState& state, const VehicleParams& params, std::string* reason);
    void abort(const std::string& reason);

    ParkingCommand update(const VehicleState& state, const VehicleParams& params, Gear actualGear,
                          const SensorReadings& sensors, double maxBrakeDecel, double dt);

    ParkingPhase phase() const { return phase_; }
    const ParkingPlan& plan() const { return plan_; }
    int slotId() const { return plan_.slotId; }
    std::string statusText() const;
    int planRevision() const { return planRevision_; }  // increments whenever the shown plan changes
    std::vector<Event> takeEvents();

    // Exposed for tests/tools.
    ParkingPlan planFor(const SlotReading& slot, const VehicleState& state, const VehicleParams& params,
                        const SensorReadings& sensors) const;
    static bool insideSlot(const VehicleState& state, const VehicleParams& params, const SlotReading& slot,
                           double tolerance = 0.0);

private:
    void note(Severity severity, const std::string& message);
    void replan(const VehicleState& state, const VehicleParams& params, const SensorReadings& sensors);
    double headingHold(double headingDesired, const VehicleState& state, const VehicleParams& params) const;
    bool pathBlocked(const std::vector<ParkingPose>& path, const VehicleState& state, const VehicleParams& params,
                     const SensorReadings& sensors) const;
    void enter(ParkingPhase phase, const std::string& message);

    ParkingConfig config_;
    Road road_;
    ParkingPhase phase_ = ParkingPhase::Off;
    ParkingPlan plan_;        // current candidate (searching/ready) or the plan being executed
    std::string searchNote_;  // why the nearest free bay is not usable yet
    std::vector<Event> events_;
    double time_ = 0.0;
    double startTime_ = 0.0;
    double nextPlanTime_ = 0.0;
    int planRevision_ = 0;
    std::size_t progress_ = 0;
};

}  // namespace driveforge
