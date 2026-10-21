#pragma once

#include <functional>
#include <string>
#include <vector>

#include "Road.h"
#include "Sensors.h"
#include "VehicleModel.h"

namespace driveforge {

enum class OvertakePhase {
    Idle,
    ChangingOut,   // moving into the passing lane
    Passing,       // driving past the slower car / obstacle
    ChangingBack,  // returning to the original lane
};

struct OvertakeConfig {
    double minSpeed = 8.0;            // m/s: no overtaking decisions below this
    double obstacleMinSpeed = 2.5;    // m/s: avoiding a stationary obstacle still works this slowly (waiting for a gap means slowing down)
    double slowerBy = 3.0;            // m/s: a lead this much slower than the set speed is worth passing
    double decisionGapMin = 45.0;     // m
    double decisionHeadway = 3.0;     // s: consider passing once the lead is within max(decisionGapMin, headway * speed)
    double persistTime = 1.0;         // s the situation must last before acting
    double obstacleTriggerGap = 140.0;  // m: start avoiding a stationary obstacle once it is this close
    double obstacleClearTime = 2.5;   // s of driving needed to get sideways clear (a lane change takes 4 s, but the car is braking meanwhile): the gap must be at least speed * this + 8 m, otherwise braking is the only option
    double targetLaneClearGap = 80.0; // m: the lane to move into must have no slower vehicle or obstacle this close ahead
                                      // (and at least as far as the thing being passed, plus targetLaneMargin)
    double targetLaneMargin = 30.0;   // m
    double returnRetry = 0.5;         // s between attempts to move back
    double passTimeout = 45.0;        // s: give up returning if it takes longer
    double cooldown = 4.0;            // s between manoeuvres
};

struct OvertakeNote {
    bool warning = false;
    std::string text;
};

struct OvertakeOutput {
    int requestDir = 0;  // -1 / +1: please start a lane change (the caller reports back with notifyStarted)
    std::vector<OvertakeNote> notes;
};

// Autonomous-mode decisions: overtake a slower vehicle, or change lane to avoid a stationary obstacle, then return to the
// original lane. It only decides and asks; the lane-keeping controller performs the manoeuvre, and ACC / AEB stay active
// (if no lane is free, ACC slows the car to a stop behind the obstacle, and AEB brakes if it gets critical).
class Overtaking {
public:
    // Would a lane change in this direction be allowed right now? Supplied by the caller (lane keeping's own safety rules).
    using CanChange = std::function<bool(int dir)>;

    explicit Overtaking(OvertakeConfig config = {}, Road road = {}) : config_(config), road_(road) {}

    void reset();
    // The lane the car is "at home" in: it goes back there after any manoeuvre. Normally the lane it was in when Autonomous
    // mode started; updated when the driver changes lane by hand.
    void setHomeLane(int lane) { homeLane_ = lane; }
    OvertakeOutput update(const VehicleState& state, const SensorReadings& sensors, int currentLane, bool laneChanging,
                          double setSpeed, double dt, const CanChange& canChange);
    void notifyStarted(bool accepted);

    OvertakePhase phase() const { return phase_; }
    std::string statusText() const;

private:
    // obstacleCase: the lane must only be free of stationary things (a moving car ahead is fine, ACC will follow it, and
    // that beats stopping). Otherwise (overtaking a slower car) it must also be free of slower traffic, or there is no point.
    int pickDirection(const SensorReadings& sensors, int currentLane, double setSpeed, double leadGap, bool obstacleCase,
                      const CanChange& canChange) const;
    bool laneBlockedAhead(const SensorReadings& sensors, int lane, double setSpeed, double clearAhead, bool obstacleCase) const;

    OvertakeConfig config_;
    Road road_;
    OvertakePhase phase_ = OvertakePhase::Idle;
    double time_ = 0.0;
    double cooldownUntil_ = 0.0;
    double persist_ = 0.0;
    double retryAt_ = 0.0;
    double phaseStart_ = 0.0;
    double passStart_ = 0.0;
    int originLane_ = 0;
    int homeLane_ = -1;
    int pendingOrigin_ = 0;
    int targetId_ = -1;
    int pendingTarget_ = -1;
    bool pendingObstacle_ = false;
    bool obstacle_ = false;
    int pendingBack_ = 0;
    bool returning_ = false;
    int warnedObstacle_ = -1;
    double lastGap_ = 1e9;
    bool sawBehind_ = false;
};

}  // namespace driveforge
