#include "Overtaking.h"

#include <algorithm>
#include <cmath>

namespace driveforge {

void Overtaking::reset() {
    phase_ = OvertakePhase::Idle;
    persist_ = 0.0;
    targetId_ = pendingTarget_ = -1;
    pendingBack_ = 0;
    returning_ = false;
    sawBehind_ = false;
    lastGap_ = 1e9;
    homeLane_ = -1;          // learnt again from the lane the car is in
    cooldownUntil_ = time_;  // may act again straight away after a reset
}

std::string Overtaking::statusText() const {
    switch (phase_) {
    case OvertakePhase::Idle: return {};
    case OvertakePhase::ChangingOut: return obstacle_ ? "changing lane to avoid obstacle" : "changing lane to overtake";
    case OvertakePhase::Passing: return obstacle_ ? "passing obstacle" : "overtaking";
    case OvertakePhase::ChangingBack: return "returning to lane";
    }
    return {};
}

// Something ahead in `lane` that makes it a bad (or impossible) place to go. obstacleCase: only stationary things count.
bool Overtaking::laneBlockedAhead(const SensorReadings& sensors, int lane, double setSpeed, double clearAhead, bool obstacleCase) const {
    const double center = road_.laneCenter(lane);
    for (const Detection& d : sensors.detections) {
        if (d.type == ObjectType::Pedestrian || d.gap < 0.0 || d.gap > clearAhead) continue;
        if (std::abs(d.worldX - center) >= road_.laneWidth / 2.0) continue;
        const bool stationary = d.type == ObjectType::Obstacle || d.objectSpeed < 1.0;
        if (obstacleCase ? stationary : d.objectSpeed < setSpeed - config_.slowerBy) return true;
    }
    return false;
}

int Overtaking::pickDirection(const SensorReadings& sensors, int currentLane, double setSpeed, double leadGap, bool obstacleCase,
                              const CanChange& canChange) const {
    // The new lane must be free at least as far ahead as the thing we are passing (plus a margin), not just nearby.
    const double clearAhead = std::max(config_.targetLaneClearGap, leadGap + config_.targetLaneMargin);
    // Prefer the way back towards the home lane; otherwise the left (passing) lane first.
    int order[2] = {-1, +1};
    if (homeLane_ >= 0 && currentLane != homeLane_) {
        const int towardHome = homeLane_ > currentLane ? +1 : -1;
        order[0] = towardHome;
        order[1] = -towardHome;
    }
    for (int dir : order) {
        const int target = currentLane + dir;
        if (target < 0 || target >= road_.laneCount) continue;
        if (!canChange(dir)) continue;
        if (laneBlockedAhead(sensors, target, setSpeed, clearAhead, obstacleCase)) continue;
        return dir;
    }
    return 0;
}

void Overtaking::notifyStarted(bool accepted) {
    if (!accepted) {
        cooldownUntil_ = time_ + 2.0;  // do not retry every step
        pendingTarget_ = -1;
        pendingBack_ = 0;
        return;
    }
    if (pendingBack_ != 0) {
        phase_ = OvertakePhase::ChangingBack;
        phaseStart_ = time_;
        pendingBack_ = 0;
        return;
    }
    phase_ = OvertakePhase::ChangingOut;
    phaseStart_ = time_;
    originLane_ = pendingOrigin_;
    targetId_ = pendingTarget_;
    obstacle_ = pendingObstacle_;
    returning_ = false;
    sawBehind_ = false;
    lastGap_ = 1e9;
    passStart_ = time_;
}

OvertakeOutput Overtaking::update(const VehicleState& s, const SensorReadings& sensors, int currentLane, bool laneChanging,
                                  double setSpeed, double dt, const CanChange& canChange) {
    OvertakeOutput out;
    time_ += dt;
    const double v = s.speed;

    // Nearest vehicle or obstacle ahead in our own lane.
    const Detection* lead = nullptr;
    const double center = road_.laneCenter(currentLane);
    for (const Detection& d : sensors.detections) {
        if (d.type == ObjectType::Pedestrian || d.gap < 0.0) continue;
        if (std::abs(d.worldX - center) < road_.laneWidth / 2.0) { lead = &d; break; }
    }

    if (homeLane_ < 0) homeLane_ = currentLane;  // first call after Autonomous mode started

    // A stationary obstacle ahead has priority over everything else, in every phase and regardless of cooldowns:
    // find a free lane and move over. (A lane change already under way is allowed to finish first.)
    const bool stationaryLead = lead && (lead->type == ObjectType::Obstacle || lead->objectSpeed < 1.0);
    if (stationaryLead && !laneChanging && v >= config_.obstacleMinSpeed &&
        (phase_ == OvertakePhase::Idle || phase_ == OvertakePhase::Passing)) {
        const bool close = lead->gap <= config_.obstacleTriggerGap;
        const bool possible = lead->gap >= v * config_.obstacleClearTime + 8.0;  // otherwise only braking can help
        if (close && possible) {
            const int dir = pickDirection(sensors, currentLane, setSpeed, lead->gap, true, canChange);
            if (dir != 0) {
                pendingOrigin_ = currentLane;
                pendingTarget_ = lead->id;
                pendingObstacle_ = true;
                persist_ = 0.0;
                out.requestDir = dir;
                out.notes.push_back({false, std::string("Autonomous: changing lane ") + (dir < 0 ? "left" : "right") + " to avoid the obstacle"});
                return out;
            }
            if (warnedObstacle_ != lead->id) {
                warnedObstacle_ = lead->id;
                out.notes.push_back({true, "Autonomous: obstacle ahead and no free lane yet: slowing down and waiting for a gap"});
            }
        }
    }

    switch (phase_) {
    case OvertakePhase::Idle: {
        if (laneChanging || v < config_.minSpeed) { persist_ = 0.0; break; }

        // Not in the home lane (after some manoeuvre)? Head back, one lane at a time, when it is free and nothing stationary is there.
        if (time_ >= cooldownUntil_ && currentLane != homeLane_ && !stationaryLead) {
            const int back = homeLane_ > currentLane ? +1 : -1;
            const double clearAhead = std::max(config_.targetLaneClearGap, lead ? lead->gap + config_.targetLaneMargin : 0.0);
            if (canChange(back) && !laneBlockedAhead(sensors, currentLane + back, setSpeed, clearAhead, false)) {
                pendingBack_ = back;
                out.requestDir = back;
                out.notes.push_back({false, "Autonomous: returning to the original lane"});
                break;
            }
        }

        if (time_ < cooldownUntil_ || !lead || stationaryLead) { persist_ = 0.0; break; }
        const bool slower = lead->objectSpeed <= setSpeed - config_.slowerBy;
        const bool near = lead->gap <= std::max(config_.decisionGapMin, v * config_.decisionHeadway);
        const bool notPullingAway = lead->closingSpeed > -0.5;
        if (!(slower && near && notPullingAway)) { persist_ = 0.0; break; }
        persist_ += dt;
        if (persist_ < config_.persistTime) break;
        const int dir = pickDirection(sensors, currentLane, setSpeed, lead->gap, false, canChange);
        if (dir == 0) break;  // keep following; try again as traffic changes
        persist_ = 0.0;
        pendingOrigin_ = currentLane;
        pendingTarget_ = lead->id;
        pendingObstacle_ = false;
        out.requestDir = dir;
        out.notes.push_back({false, std::string("Autonomous: overtaking a slower vehicle on the ") + (dir < 0 ? "left" : "right")});
        break;
    }
    case OvertakePhase::ChangingOut:
        if (!laneChanging && time_ - phaseStart_ > 0.3) {
            phase_ = OvertakePhase::Passing;
            passStart_ = time_;
        } else if (time_ - phaseStart_ > 15.0) {
            phase_ = OvertakePhase::Idle;
            cooldownUntil_ = time_ + config_.cooldown;
        }
        break;
    case OvertakePhase::Passing: {
        if (time_ - passStart_ > config_.passTimeout) {
            phase_ = OvertakePhase::Idle;
            cooldownUntil_ = time_ + config_.cooldown;
            out.notes.push_back({false, "Autonomous: staying in the new lane"});
            break;
        }
        // Where is the vehicle / obstacle we are passing?
        bool found = false;
        for (const Detection& d : sensors.detections) {
            if (d.id != targetId_) continue;
            found = true;
            lastGap_ = d.gap;
            if (d.gap < 0.0) sawBehind_ = true;
            break;
        }
        const bool passed = found ? lastGap_ < 0.0 : sawBehind_;  // out of range behind us counts as passed
        if (!passed || time_ < retryAt_) break;
        const int back = homeLane_ > currentLane ? +1 : homeLane_ < currentLane ? -1 : 0;
        if (back == 0) {
            phase_ = OvertakePhase::Idle;
            cooldownUntil_ = time_ + config_.cooldown;
            break;
        }
        if (laneChanging) break;
        if (canChange(back)) {
            pendingBack_ = back;
            out.requestDir = back;
            out.notes.push_back({false, "Autonomous: returning to the original lane"});
        }
        retryAt_ = time_ + config_.returnRetry;
        break;
    }
    case OvertakePhase::ChangingBack:
        if (!laneChanging && time_ - phaseStart_ > 0.3) {
            phase_ = OvertakePhase::Idle;
            cooldownUntil_ = time_ + config_.cooldown;
            out.notes.push_back({false, "Autonomous: manoeuvre complete"});
        } else if (time_ - phaseStart_ > 15.0) {
            phase_ = OvertakePhase::Idle;
            cooldownUntil_ = time_ + config_.cooldown;
        }
        break;
    }
    return out;
}

}  // namespace driveforge
