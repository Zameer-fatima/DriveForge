#pragma once

#include <string>

#include "Scenario.h"
#include "Simulation.h"

namespace driveforge {

// Sets up a scenario on a Simulation, scripts the other actors and the driver, scores the run, and builds the report.
// Usage per step:  beforeStep(sim); sim.step(dt); afterStep(sim);
// Deterministic: the same scenario and parameters always produce identical reports.
class ScenarioRunner {
public:
    void start(Simulation& sim, ScenarioId id, const ParamMap& overrides = {});
    void beforeStep(Simulation& sim);
    void afterStep(Simulation& sim);
    void abort(Simulation& sim, const std::string& reason);

    bool running() const { return running_; }
    bool finished() const { return finished_; }
    double elapsed() const { return elapsed_; }
    double timeLimit() const { return timeLimit_; }
    ScenarioId id() const { return id_; }
    const ScenarioReport& report() const { return report_; }
    std::string statusText() const;

    // Runs a scenario to completion on a fresh Simulation (fixed 1/120 s steps).
    static ScenarioReport runHeadless(ScenarioId id, const ParamMap& overrides = {});

private:
    void finish(Simulation& sim, const std::string& reason, bool aborted);
    void evaluate(Simulation& sim, bool timedOut);
    void track(Simulation& sim);
    void add(const std::string& name, const std::string& requirement, const std::string& measured, bool passed);

    ScenarioId id_ = ScenarioId::Highway;
    ParamMap p_;
    bool running_ = false;
    bool finished_ = false;
    double elapsed_ = 0.0;
    double timeLimit_ = 0.0;
    ScenarioReport report_;

    int targetId_ = -1;  // lead car / obstacle / pedestrian being scored against
    int targetLane_ = 1;
    int egoLane_ = 1;
    double leadSpeed_ = 0.0;
    bool wasParkingActive_ = false;

    // Metrics
    double minSeparation_ = 1e9;
    double maxLaneOffset_ = 0.0;
    double maxLateral_ = 0.0;
    double maxDecel_ = 0.0;
    double stoppedFor_ = 0.0;
    double aebFirstTime_ = -1.0;
    bool aebBraked_ = false;
    bool autonomousDropped_ = false;
    bool aebWasArmed_ = true;

    // Overtaking / lane-avoid scenarios
    bool manoeuvreSeen_ = false;
    double idleSince_ = -1.0;
    bool obstacleSpawned_ = false;
    double minSpeed_ = 1e9;
    // Last known position of the scored target: the world removes things far behind the car, so remember it.
    double targetY_ = 0.0;
    bool targetSeen_ = false;
};

}  // namespace driveforge
