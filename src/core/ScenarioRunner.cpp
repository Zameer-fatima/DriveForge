#include "ScenarioRunner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace driveforge {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kKmh = 1.0 / 3.6;
constexpr double kStep = 1.0 / 120.0;

std::string fmt(const char* f, double a, double b = 0.0) {
    char buf[128];
    std::snprintf(buf, sizeof buf, f, a, b);
    return buf;
}
}  // namespace

void ScenarioRunner::add(const std::string& name, const std::string& requirement, const std::string& measured, bool passed) {
    report_.criteria.push_back({name, requirement, measured, passed});
}

void ScenarioRunner::start(Simulation& sim, ScenarioId id, const ParamMap& overrides) {
    id_ = id;
    p_ = resolveParams(id, overrides);
    sim.reset();

    report_ = ScenarioReport{};
    report_.id = id;
    report_.name = scenarioInfo(id).name;
    report_.params = p_;
    running_ = true;
    finished_ = false;
    elapsed_ = 0.0;
    targetId_ = -1;
    wasParkingActive_ = false;
    minSeparation_ = 1e9;
    maxLaneOffset_ = maxLateral_ = maxDecel_ = stoppedFor_ = 0.0;
    aebFirstTime_ = -1.0;
    aebBraked_ = autonomousDropped_ = false;
    aebWasArmed_ = true;
    manoeuvreSeen_ = false;
    idleSince_ = -1.0;
    obstacleSpawned_ = false;
    minSpeed_ = 1e9;
    targetY_ = 0.0;
    targetSeen_ = false;

    const Road& road = sim.world().road();
    const double egoLen = sim.vehicle().params().length;
    sim.driver() = ControlInput{};
    sim.driver().ignition = true;

    switch (id) {
    case ScenarioId::Highway: {
        timeLimit_ = p_["duration"];
        sim.vehicle().reset(0.0, 0.0, 0.0, p_["egoSpeed"] * kKmh);
        egoLane_ = road.laneIndexAt(0.0);
        leadSpeed_ = p_["leadSpeed"] * kKmh;
        targetId_ = sim.world().add(World::makeVehicle(road.laneCenter(egoLane_), p_["gap"] + egoLen, leadSpeed_));
        sim.adas().setAutonomousSetpoint(p_["accSpeed"] * kKmh, p_["timeGap"]);
        sim.adas().request(AdasRequest::ToggleOvertake);  // this scenario tests following, so no overtaking
        sim.adas().request(AdasRequest::ToggleAutonomous);
        break;
    }
    case ScenarioId::Obstacle: {
        timeLimit_ = 60.0;
        sim.vehicle().reset(0.0, 0.0, 0.0, p_["speed"] * kKmh);
        egoLane_ = road.laneIndexAt(0.0);
        targetLane_ = static_cast<int>(std::lround(p_["obstacleLane"]));
        targetId_ = sim.world().add(World::makeObstacle(road.laneCenter(targetLane_), p_["distance"] + egoLen / 2.0 + 0.5));
        sim.driver().throttle = p_["driverThrottle"];
        aebWasArmed_ = p_["aebArmed"] >= 0.5;
        if (!aebWasArmed_) sim.adas().request(AdasRequest::ToggleAeb);
        break;
    }
    case ScenarioId::Pedestrian: {
        timeLimit_ = 45.0;
        sim.vehicle().reset(0.0, 0.0, 0.0, p_["speed"] * kKmh);
        egoLane_ = road.laneIndexAt(0.0);
        targetId_ = sim.world().add(
            World::makePedestrian(road.halfWidth() + 1.5, p_["distance"], -p_["pedSpeed"], p_["triggerTtc"]));
        aebWasArmed_ = p_["aebArmed"] >= 0.5;
        if (!aebWasArmed_) sim.adas().request(AdasRequest::ToggleAeb);
        if (p_["autonomous"] >= 0.5) {
            sim.adas().setAutonomousSetpoint(p_["speed"] * kKmh, 1.8);
            sim.adas().request(AdasRequest::ToggleAutonomous);
        } else {
            sim.driver().throttle = p_["driverThrottle"];
        }
        break;
    }
    case ScenarioId::Parking: {
        timeLimit_ = p_["timeout"];
        sim.vehicle().reset(p_["startX"], 0.0, 0.0, p_["startSpeed"] * kKmh);
        const int variant = static_cast<int>(std::lround(p_["freeBays"]));
        std::vector<bool> occupied = {true, true, true, variant == 3, true, true, variant == 2, true};
        sim.world().addParkingRow(p_["bayDistance"] - 3.0 * ParkingSlot{}.width, 8, occupied);
        sim.adas().request(AdasRequest::StartParking);
        break;
    }
    case ScenarioId::Overtake: {
        timeLimit_ = p_["duration"];
        sim.vehicle().reset(0.0, 0.0, 0.0, p_["accSpeed"] * kKmh);
        egoLane_ = road.laneIndexAt(0.0);
        targetId_ = sim.world().add(World::makeVehicle(road.laneCenter(egoLane_), p_["gap"] + egoLen, p_["leadSpeed"] * kKmh));
        if (p_["blockLeft"] >= 0.5 && egoLane_ > 0)  // keeps pace beside us, so the left lane is not available
            sim.world().add(World::makeVehicle(road.laneCenter(egoLane_ - 1), 0.0, p_["accSpeed"] * kKmh));
        sim.adas().setAutonomousSetpoint(p_["accSpeed"] * kKmh, 1.8);
        sim.adas().request(AdasRequest::ToggleAutonomous);
        break;
    }
    case ScenarioId::LaneAvoid: {
        timeLimit_ = p_["duration"];
        sim.vehicle().reset(0.0, 0.0, 0.0, p_["speed"] * kKmh);
        egoLane_ = road.laneIndexAt(0.0);
        if (p_["blockSides"] >= 0.5) {
            if (egoLane_ > 0) sim.world().add(World::makeVehicle(road.laneCenter(egoLane_ - 1), 0.0, p_["speed"] * kKmh));
            if (egoLane_ + 1 < road.laneCount) sim.world().add(World::makeVehicle(road.laneCenter(egoLane_ + 1), 0.0, p_["speed"] * kKmh));
        }
        sim.adas().setAutonomousSetpoint(p_["speed"] * kKmh, 1.8);
        sim.adas().request(AdasRequest::ToggleAutonomous);
        break;
    }
    }
}

void ScenarioRunner::beforeStep(Simulation& sim) {
    if (!running_) return;
    if (id_ == ScenarioId::Highway) {
        // Lead car brakes at the scripted time, down to its final speed.
        if (elapsed_ >= p_["leadBrakeAt"]) {
            leadSpeed_ = std::max(p_["leadFinalSpeed"] * kKmh, leadSpeed_ - p_["leadDecel"] * kStep);
            if (WorldObject* lead = sim.world().find(targetId_)) lead->vy = leadSpeed_;
        }
    }
    if (id_ == ScenarioId::LaneAvoid && !obstacleSpawned_ && elapsed_ >= p_["appearAt"]) {
        // The obstacle appears ahead in the car's own lane, within sensor range.
        const Road& road = sim.world().road();
        const double egoLen = sim.vehicle().params().length;
        targetId_ = sim.world().add(World::makeObstacle(road.laneCenter(egoLane_), sim.vehicle().state().y + p_["distance"] + egoLen / 2.0 + 0.5));
        obstacleSpawned_ = true;
    }
}

void ScenarioRunner::track(Simulation& sim) {
    const VehicleState& s = sim.vehicle().state();
    const AdasStatus& a = sim.adas().status();

    if (const WorldObject* target = sim.world().find(targetId_)) {
        minSeparation_ = std::min(minSeparation_, separation(s, sim.vehicle().params(), *target));
        targetY_ = target->y;
        targetSeen_ = true;
    }
    maxLateral_ = std::max(maxLateral_, std::abs(s.lateralAcceleration));
    if (s.speed > 0.5) maxDecel_ = std::max(maxDecel_, -s.acceleration);
    stoppedFor_ = std::abs(s.speed) < 0.05 ? stoppedFor_ + kStep : 0.0;
    if (a.aebBraking) {
        aebBraked_ = true;
        if (aebFirstTime_ < 0.0) aebFirstTime_ = s.time;
    }
    if (id_ == ScenarioId::LaneAvoid && obstacleSpawned_) minSpeed_ = std::min(minSpeed_, s.speed);
    if (a.overtakePhase != OvertakePhase::Idle) {
        manoeuvreSeen_ = true;
        idleSince_ = -1.0;
    } else if (manoeuvreSeen_ && idleSince_ < 0.0) {
        idleSince_ = s.time;
    }
    if (id_ == ScenarioId::Overtake || id_ == ScenarioId::LaneAvoid) {
        if (elapsed_ > 0.5 && a.mode != DriveMode::Autonomous) autonomousDropped_ = true;
    }
    if (id_ == ScenarioId::Highway) {
        const Road& road = sim.world().road();
        maxLaneOffset_ = std::max(maxLaneOffset_, std::abs(s.x - road.laneCenter(egoLane_)));
        if (elapsed_ > 0.5 && a.mode != DriveMode::Autonomous) autonomousDropped_ = true;
    }
    if (id_ == ScenarioId::Parking && a.parkingActive) wasParkingActive_ = true;
}

void ScenarioRunner::afterStep(Simulation& sim) {
    if (!running_) return;
    elapsed_ = sim.vehicle().state().time;
    track(sim);

    const VehicleState& s = sim.vehicle().state();
    if (sim.collisions() > 0) { finish(sim, "collision", false); return; }

    switch (id_) {
    case ScenarioId::Highway:
        if (elapsed_ >= timeLimit_) finish(sim, "duration reached", false);
        break;
    case ScenarioId::Obstacle: {
        const WorldObject* o = sim.world().find(targetId_);
        if (stoppedFor_ >= 2.0 && elapsed_ > 1.0) finish(sim, "vehicle stopped", false);
        else if (o && s.y > o->y + 20.0) finish(sim, "obstacle passed", false);
        else if (elapsed_ >= timeLimit_) finish(sim, "time limit", false);
        break;
    }
    case ScenarioId::Pedestrian: {
        const WorldObject* o = sim.world().find(targetId_);
        if (stoppedFor_ >= 3.0 && elapsed_ > 1.0) finish(sim, "vehicle stopped", false);
        else if (o && s.y > o->y + 15.0) finish(sim, "pedestrian passed", false);
        else if (elapsed_ >= timeLimit_) finish(sim, "time limit", false);
        break;
    }
    case ScenarioId::Overtake:
        if (manoeuvreSeen_ && idleSince_ >= 0.0 && elapsed_ - idleSince_ >= 3.0) finish(sim, "overtake complete", false);
        else if (elapsed_ >= timeLimit_) finish(sim, "time limit", false);
        break;
    case ScenarioId::LaneAvoid: {
        const WorldObject* o = obstacleSpawned_ ? sim.world().find(targetId_) : nullptr;
        if (stoppedFor_ >= 2.0 && elapsed_ > 1.0) finish(sim, "vehicle stopped", false);
        else if (manoeuvreSeen_ && idleSince_ >= 0.0 && elapsed_ - idleSince_ >= 3.0) finish(sim, "manoeuvre complete", false);
        else if (!manoeuvreSeen_ && o && s.y > o->y + 60.0) finish(sim, "obstacle passed", false);
        else if (elapsed_ >= timeLimit_) finish(sim, "time limit", false);
        break;
    }
    case ScenarioId::Parking: {
        const AdasStatus& a = sim.adas().status();
        if (a.parkingPhase == ParkingPhase::Parked) finish(sim, "parked", false);
        else if (wasParkingActive_ && !a.parkingActive) finish(sim, "parking aborted", false);
        else if (!wasParkingActive_ && elapsed_ > 15.0) finish(sim, "parking never started", false);
        else if (elapsed_ >= timeLimit_) finish(sim, "time limit", false);
        break;
    }
    }
}

void ScenarioRunner::abort(Simulation& sim, const std::string& reason) {
    if (!running_) return;
    finish(sim, reason, true);
}

void ScenarioRunner::finish(Simulation& sim, const std::string& reason, bool aborted) {
    running_ = false;
    finished_ = true;
    report_.endReason = reason;
    report_.duration = elapsed_;
    if (aborted) {
        report_.outcome = Outcome::Aborted;
    } else {
        evaluate(sim, reason == "time limit");
        bool all = !report_.criteria.empty();
        for (const Criterion& c : report_.criteria) all = all && c.passed;
        report_.outcome = all ? Outcome::Pass : Outcome::Fail;
    }

    report_.metrics["duration_s"] = elapsed_;
    report_.metrics["collisions"] = sim.collisions();
    report_.metrics["min_separation_m"] = minSeparation_ > 1e8 ? -1.0 : minSeparation_;
    report_.metrics["max_lateral_accel"] = maxLateral_;
    report_.metrics["max_decel"] = maxDecel_;
    report_.metrics["aeb_braked"] = aebBraked_ ? 1.0 : 0.0;
    report_.metrics["aeb_first_time_s"] = aebFirstTime_;
    report_.metrics["final_x"] = sim.vehicle().state().x;
    report_.metrics["final_y"] = sim.vehicle().state().y;
    report_.metrics["final_speed"] = sim.vehicle().state().speed;
    if (aebFirstTime_ >= 0.0) report_.notes.push_back(fmt("AEB first braked at t = %.1f s.", aebFirstTime_));

    // Hand control back: stop assists and brake moderately (below the hard-braking warning level).
    sim.adas().releaseAll();
    sim.driver() = ControlInput{};
    sim.driver().ignition = true;
    sim.driver().brake = 0.4;
}

void ScenarioRunner::evaluate(Simulation& sim, bool timedOut) {
    const VehicleState& s = sim.vehicle().state();
    const int collisions = sim.collisions();
    add("No collision", "0 collisions", std::to_string(collisions) + (collisions == 1 ? " collision" : " collisions"), collisions == 0);

    switch (id_) {
    case ScenarioId::Highway: {
        add("Safe following distance", ">= 2.0 m to the lead car at all times", fmt("minimum %.1f m", minSeparation_), minSeparation_ >= 2.0);
        add("Stayed in lane", "lane offset <= 0.5 m", fmt("maximum %.2f m", maxLaneOffset_), maxLaneOffset_ <= 0.5);
        add("Comfortable steering", "lateral acceleration <= 3 m/s²", fmt("maximum %.2f m/s²", maxLateral_), maxLateral_ <= 3.0);
        add("Autonomous mode stayed engaged", "no disengagement", autonomousDropped_ ? "disengaged" : "engaged throughout", !autonomousDropped_);
        report_.metrics["max_lane_offset_m"] = maxLaneOffset_;
        break;
    }
    case ScenarioId::Obstacle: {
        const bool sameLane = targetLane_ == egoLane_;
        if (sameLane) {
            add("Clearance to the obstacle", ">= 0.3 m", fmt("minimum %.2f m", minSeparation_), minSeparation_ >= 0.3);
            add("Vehicle came to a stop", "speed < 0.1 m/s", fmt("final speed %.2f m/s", s.speed), std::abs(s.speed) < 0.1);
        } else {
            add("No false emergency braking", "AEB must not brake for an object in another lane", aebBraked_ ? "AEB braked" : "no AEB braking", !aebBraked_);
        }
        if (!aebWasArmed_) report_.notes.push_back("AEB was disabled for this run.");
        break;
    }
    case ScenarioId::Pedestrian: {
        add("Clearance to the pedestrian", ">= 0.5 m", fmt("minimum %.2f m", minSeparation_), minSeparation_ >= 0.5);
        add("Braking within limits", "deceleration <= 9 m/s²", fmt("maximum %.1f m/s²", maxDecel_), maxDecel_ <= 9.0);
        if (!aebWasArmed_) report_.notes.push_back("AEB was disabled for this run.");
        break;
    }
    case ScenarioId::Overtake: {
        const Road& road = sim.world().road();
        const bool overtook = targetSeen_ && s.y > targetY_ + 5.0;
        const double offset = std::abs(s.x - road.laneCenter(egoLane_));
        add("Overtook the slower car", "ahead of it at the end", overtook ? "overtook it" : "still behind it", overtook);
        add("Returned to its original lane", "within 0.5 m of the lane centre", fmt("offset %.2f m", offset), offset <= 0.5);
        add("Safe distance while passing", ">= 1.0 m to the car at all times", fmt("minimum %.1f m", minSeparation_), minSeparation_ >= 1.0);
        add("Comfortable and smooth", "lateral acceleration <= 3 m/s², no emergency braking", fmt("max %.2f m/s², ", maxLateral_) + (aebBraked_ ? "AEB braked" : "no AEB braking"), maxLateral_ <= 3.0 && !aebBraked_);
        add("Autonomous mode stayed engaged", "no disengagement", autonomousDropped_ ? "disengaged" : "engaged throughout", !autonomousDropped_);
        report_.metrics["final_lane_offset_m"] = offset;
        report_.metrics["overtook"] = overtook ? 1.0 : 0.0;
        break;
    }
    case ScenarioId::LaneAvoid: {
        const Road& road = sim.world().road();
        const bool blocked = p_["blockSides"] >= 0.5;
        const double offset = std::abs(s.x - road.laneCenter(egoLane_));
        add("The obstacle appeared", "within the time limit", obstacleSpawned_ ? "appeared" : "never appeared", obstacleSpawned_);
        add("Clearance to the obstacle", ">= 0.5 m", fmt("minimum %.2f m", minSeparation_), obstacleSpawned_ && minSeparation_ >= 0.5);
        add("Comfortable steering", "lateral acceleration <= 3 m/s²", fmt("maximum %.2f m/s²", maxLateral_), maxLateral_ <= 3.0);
        const bool passed = targetSeen_ && s.y > targetY_;
        const bool stopped = std::abs(s.speed) < 0.1;
        const double minKmh = minSpeed_ > 1e8 ? 0.0 : minSpeed_ / kKmh;
        // Either outcome is acceptable if it is collision-free: drive around the obstacle, or stop short of it when there is no
        // room for a lane change (too close, or no free lane).
        add("Avoided the obstacle", "drove past it, or stopped short of it", passed ? fmt("drove past, slowest %.0f km/h", minKmh) : (stopped ? "stopped short of it" : "neither passed nor stopped"), passed || stopped);
        if (passed && !blocked) {
            add("No emergency braking", "AEB not needed when there is room to change lane", aebBraked_ ? "AEB braked" : "no AEB braking", !aebBraked_);
            add("Returned to its original lane", "within 0.5 m of the lane centre", fmt("offset %.2f m", offset), offset <= 0.5);
        }
        if (blocked)
            report_.notes.push_back("Side lanes were occupied: the car may slow down and change lane once a gap opens, or stop. Either is acceptable if it is collision-free.");
        report_.metrics["passed"] = passed ? 1.0 : 0.0;
        add("Autonomous mode stayed engaged", "no disengagement", autonomousDropped_ ? "disengaged" : "engaged throughout", !autonomousDropped_);
        report_.metrics["final_lane_offset_m"] = offset;
        break;
    }
    case ScenarioId::Parking: {
        const AdasStatus& a = sim.adas().status();
        const bool parked = a.parkingPhase == ParkingPhase::Parked;
        add("Parked within the time limit", fmt("parked within %.0f s", timeLimit_), parked ? fmt("parked after %.1f s", elapsed_) : (timedOut ? "time limit reached" : "not parked"), parked);
        double offset = -1.0, headingErr = -1.0;
        bool inside = false;
        if (parked) {
            for (const SlotReading& r : sim.sensors().bays) {
                if (r.id != a.parkingSlot) continue;
                offset = std::hypot(s.x - r.x, s.y - r.y);
                inside = ParkingAssist::insideSlot(s, sim.vehicle().params(), r, 0.1);
            }
            headingErr = std::abs(s.heading + kPi / 2.0);
        }
        add("Final position", fmt("within %.1f m of the bay centre", p_["maxOffset"]), offset >= 0.0 ? fmt("offset %.2f m", offset) : "n/a", offset >= 0.0 && offset <= p_["maxOffset"]);
        add("Final heading", "within 5° of perpendicular", headingErr >= 0.0 ? fmt("error %.1f°", headingErr * 180.0 / kPi) : "n/a", headingErr >= 0.0 && headingErr * 180.0 / kPi <= 5.0);
        add("Inside the bay lines", "all corners inside (0.1 m tolerance)", parked ? (inside ? "inside" : "outside") : "n/a", inside);
        report_.metrics["final_offset_m"] = offset;
        report_.metrics["final_heading_error_deg"] = headingErr >= 0.0 ? headingErr * 180.0 / kPi : -1.0;
        break;
    }
    }
}

std::string ScenarioRunner::statusText() const {
    if (running_) return scenarioInfo(id_).name + ": running " + fmt("%.1f / %.0f s", elapsed_, timeLimit_);
    if (finished_) return scenarioInfo(id_).name + ": " + report_.outcomeText();
    return "no scenario";
}

ScenarioReport ScenarioRunner::runHeadless(ScenarioId id, const ParamMap& overrides) {
    Simulation sim;
    ScenarioRunner runner;
    runner.start(sim, id, overrides);
    const long maxSteps = static_cast<long>((runner.timeLimit_ + 30.0) / kStep);
    for (long i = 0; i < maxSteps && runner.running(); ++i) {
        runner.beforeStep(sim);
        sim.step(kStep);
        runner.afterStep(sim);
    }
    if (runner.running()) runner.abort(sim, "step limit");
    return runner.report();
}

}  // namespace driveforge
