#include "Parking.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "Longitudinal.h"
#include "World.h"

namespace driveforge {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;

std::string format(const char* fmt, double a, double b = 0.0, double c = 0.0) {
    char buf[160];
    std::snprintf(buf, sizeof buf, fmt, a, b, c);
    return buf;
}

WorldObject boxOf(const Detection& d) {
    WorldObject o;
    o.x = d.worldX;
    o.y = d.worldY;
    o.width = d.halfWidth * 2.0;
    o.length = d.halfLength * 2.0;
    return o;
}

VehicleState poseState(const ParkingPose& p) {
    VehicleState s;
    s.x = p.x;
    s.y = p.y;
    s.heading = p.heading;
    return s;
}

VehicleParams inflated(const VehicleParams& p, double margin) {
    VehicleParams q = p;
    q.length += 2.0 * margin;
    q.width += 2.0 * margin;
    return q;
}

// Smallest margin level (0 .. 0.6 m) at which the path still touches nothing; -1 if it touches at 0.
double pathClearance(const std::vector<ParkingPose>& path, const VehicleParams& p, const SensorReadings& sensors) {
    std::vector<WorldObject> boxes;
    for (const Detection& d : sensors.detections) boxes.push_back(boxOf(d));
    double best = -1.0;
    for (double margin = 0.0; margin <= 0.6001; margin += 0.05) {
        const VehicleParams q = inflated(p, margin);
        bool clear = true;
        for (const ParkingPose& pose : path) {
            const VehicleState s = poseState(pose);
            for (const WorldObject& b : boxes)
                if (overlaps(s, q, b)) { clear = false; break; }
            if (!clear) break;
        }
        if (!clear) break;
        best = margin;
    }
    return best;
}

// Poses along: reverse arc (heading 0 -> -90 deg, right steering) from (x0, y0), then straight to (xf, yf).
std::vector<ParkingPose> buildPath(double x0, double y0, double R, double xf, double yf) {
    std::vector<ParkingPose> path;
    const double step = 0.25;
    const int arcSteps = std::max(2, static_cast<int>(std::ceil(R * kHalfPi / step)));
    for (int i = 0; i <= arcSteps; ++i) {
        const double psi = -kHalfPi * i / arcSteps;
        path.push_back({x0 + R * (1.0 - std::cos(psi)), y0 + R * std::sin(psi), psi});
    }
    const double xs = x0 + R;
    const int lineSteps = static_cast<int>(std::ceil((xf - xs) / step));
    for (int i = 1; i <= lineSteps; ++i)
        path.push_back({std::min(xs + i * step, xf), yf, -kHalfPi});
    return path;
}
}  // namespace

void ParkingAssist::reset() {
    phase_ = ParkingPhase::Off;
    plan_ = ParkingPlan{};
    searchNote_.clear();
    events_.clear();
    time_ = startTime_ = nextPlanTime_ = 0.0;
    progress_ = 0;
    ++planRevision_;
}

bool ParkingAssist::active() const {
    switch (phase_) {
    case ParkingPhase::Align:
    case ParkingPhase::PreSteer:
    case ParkingPhase::Arc:
    case ParkingPhase::Straighten:
    case ParkingPhase::Straight:
    case ParkingPhase::Settle: return true;
    default: return false;
    }
}

void ParkingAssist::note(Severity severity, const std::string& message) { events_.push_back({time_, severity, message}); }

std::vector<Event> ParkingAssist::takeEvents() {
    std::vector<Event> out;
    out.swap(events_);
    return out;
}

void ParkingAssist::setEnabled(bool enabled) {
    if (enabled == this->enabled()) return;
    plan_ = ParkingPlan{};
    searchNote_.clear();
    nextPlanTime_ = 0.0;
    ++planRevision_;
    phase_ = enabled ? ParkingPhase::Searching : ParkingPhase::Off;
    note(Severity::Info, enabled ? "Parking assist ON: searching for a free bay" : "Parking assist OFF");
}

void ParkingAssist::enter(ParkingPhase phase, const std::string& message) {
    phase_ = phase;
    if (!message.empty()) note(Severity::Info, message);
}

void ParkingAssist::abort(const std::string& reason) {
    if (!active()) return;
    note(Severity::Warning, "Parking aborted: " + reason);
    plan_ = ParkingPlan{};
    ++planRevision_;
    nextPlanTime_ = 0.0;
    phase_ = ParkingPhase::Searching;
}

bool ParkingAssist::insideSlot(const VehicleState& s, const VehicleParams& p, const SlotReading& slot, double tol) {
    const double ux = std::sin(s.heading), uy = std::cos(s.heading);
    const double rx = uy, ry = -ux;
    for (int i = 0; i < 4; ++i) {
        const double a = (i & 1 ? 1.0 : -1.0) * p.length / 2.0;
        const double b = (i & 2 ? 1.0 : -1.0) * p.width / 2.0;
        const double cx = s.x + a * ux + b * rx, cy = s.y + a * uy + b * ry;
        if (std::abs(cx - slot.x) > slot.length / 2.0 + tol || std::abs(cy - slot.y) > slot.width / 2.0 + tol) return false;
    }
    return true;
}

ParkingPlan ParkingAssist::planFor(const SlotReading& slot, const VehicleState& s, const VehicleParams& p,
                                   const SensorReadings& sensors) const {
    ParkingPlan plan;
    plan.slotId = slot.id;
    plan.xf = slot.x;
    plan.yf = slot.y;
    if (std::abs(s.heading) > config_.headingTol) {
        plan.reason = "drive straight, parallel to the road";
        return plan;
    }
    const double rMin = p.wheelbase / std::tan(config_.steerUsage * p.maxSteerAngle);
    const double rMax = slot.x - s.x;
    if (rMax < rMin) {
        plan.reason = "too close to the bays, move towards the road";
        return plan;
    }

    double bestScore = -1.0;
    for (double R = rMin; R <= rMax + 1e-9; R += config_.radiusStep) {
        const double x0 = s.x, y0 = slot.y + R;
        std::vector<ParkingPose> path = buildPath(x0, y0, R, slot.x, slot.y);
        const double clearance = pathClearance(path, p, sensors);
        if (clearance < config_.clearance) continue;
        const double score = clearance - 0.001 * std::abs(R - 0.5 * (rMin + rMax));  // prefer margin, then mid-range radius
        if (score > bestScore) {
            bestScore = score;
            plan.valid = true;
            plan.x0 = x0;
            plan.y0 = y0;
            plan.radius = R;
            plan.straight = slot.x - (x0 + R);
            plan.steerAngle = std::atan(p.wheelbase / R);
            plan.clearance = clearance;
            plan.path = std::move(path);
        }
    }
    if (!plan.valid) plan.reason = "no collision-free path from here, try a different lateral position";
    return plan;
}

void ParkingAssist::replan(const VehicleState& s, const VehicleParams& p, const SensorReadings& sensors) {
    ParkingPlan best;
    std::string firstReason;
    double bestDistance = 1e9;
    for (const SlotReading& slot : sensors.bays) {
        if (!slot.free) continue;
        ParkingPlan candidate = planFor(slot, s, p, sensors);
        if (!candidate.valid) {
            if (firstReason.empty()) firstReason = format("bay %.0f: ", slot.id) + candidate.reason;
            continue;
        }
        const double d = std::abs(candidate.y0 - s.y);
        if (d < bestDistance) {
            bestDistance = d;
            best = std::move(candidate);
        }
    }

    const bool wasReady = phase_ == ParkingPhase::Ready;
    const int oldSlot = plan_.slotId;
    // The shown plan only counts as changed when its geometry changes, so the UI does not rebuild the path markers every replan.
    const bool same = best.valid == plan_.valid && best.slotId == plan_.slotId && std::abs(best.x0 - plan_.x0) < 0.02 &&
                      std::abs(best.y0 - plan_.y0) < 0.02 && std::abs(best.radius - plan_.radius) < 0.02;
    if (best.valid) {
        plan_ = std::move(best);
        phase_ = ParkingPhase::Ready;
        searchNote_.clear();
        if (!wasReady || oldSlot != plan_.slotId)
            note(Severity::Info, format("Bay %.0f is free: ready to park (R %.1f m). Press Y to start", plan_.slotId, plan_.radius));
    } else {
        if (wasReady) note(Severity::Info, "Parking assist: bay no longer reachable");
        plan_ = ParkingPlan{};
        phase_ = ParkingPhase::Searching;
        searchNote_ = firstReason;
    }
    if (!same) ++planRevision_;
}

bool ParkingAssist::start(const VehicleState& s, const VehicleParams&, std::string* reason) {
    auto refuse = [&](const char* why) {
        if (reason) *reason = why;
        return false;
    };
    if (phase_ != ParkingPhase::Ready || !plan_.valid) return refuse("no bay ready");
    if (std::abs(s.speed) > config_.maxStartSpeed) return refuse("slow down below 100 km/h first");
    startTime_ = time_;
    progress_ = 0;
    enter(ParkingPhase::Align, format("Parking into bay %.0f: aligning", plan_.slotId));
    return true;
}

std::string ParkingAssist::statusText() const {
    switch (phase_) {
    case ParkingPhase::Off: return "off";
    case ParkingPhase::Searching: return searchNote_.empty() ? "searching for a free bay" : "no usable bay (" + searchNote_ + ")";
    case ParkingPhase::Ready: return format("bay %.0f ready, press Y to park", plan_.slotId);
    case ParkingPhase::Align: return format("bay %.0f: aligning", plan_.slotId);
    case ParkingPhase::PreSteer: return format("bay %.0f: turning wheels", plan_.slotId);
    case ParkingPhase::Arc: return format("bay %.0f: reversing along arc", plan_.slotId);
    case ParkingPhase::Straighten: return format("bay %.0f: straightening wheels", plan_.slotId);
    case ParkingPhase::Straight: return format("bay %.0f: reversing into bay", plan_.slotId);
    case ParkingPhase::Settle: return format("bay %.0f: settling", plan_.slotId);
    case ParkingPhase::Parked: return format("parked in bay %.0f", plan_.slotId);
    }
    return "";
}

// Steering that rotates the heading towards headingDesired; valid for forward and reverse motion.
double ParkingAssist::headingHold(double headingDesired, const VehicleState& s, const VehicleParams& p) const {
    if (std::abs(s.speed) < 0.15) return 0.0;
    const double yawRateDesired = 1.5 * (headingDesired - s.heading);
    const double delta = std::atan(p.wheelbase * yawRateDesired / s.speed);
    return std::clamp(delta / p.maxSteerAngle, -1.0, 1.0);
}

bool ParkingAssist::pathBlocked(const std::vector<ParkingPose>& path, const VehicleState& s, const VehicleParams& p,
                                const SensorReadings& sensors) const {
    if (path.empty() || sensors.detections.empty()) return false;
    const VehicleParams q = inflated(p, config_.abortClearance);
    // Walk forward along the plan from the pose nearest to the car.
    std::size_t nearest = progress_;
    double bestD = 1e9;
    for (std::size_t i = progress_; i < path.size(); ++i) {
        const double d = std::hypot(path[i].x - s.x, path[i].y - s.y);
        if (d < bestD) { bestD = d; nearest = i; }
        else if (d > bestD + 1.0) break;
    }
    double travelled = 0.0;
    for (std::size_t i = nearest; i < path.size() && travelled <= config_.lookahead; ++i) {
        if (i > nearest) travelled += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
        const VehicleState pose = poseState(path[i]);
        for (const Detection& d : sensors.detections)
            if (overlaps(pose, q, boxOf(d))) return true;
    }
    return false;
}

ParkingCommand ParkingAssist::update(const VehicleState& s, const VehicleParams& p, Gear gear,
                                     const SensorReadings& sensors, double maxBrake, double dt) {
    (void)dt;
    time_ = s.time;
    ParkingCommand cmd;
    if (phase_ == ParkingPhase::Off || phase_ == ParkingPhase::Parked) return cmd;

    if (phase_ == ParkingPhase::Searching || phase_ == ParkingPhase::Ready) {
        if (time_ >= nextPlanTime_) {
            replan(s, p, sensors);
            nextPlanTime_ = time_ + config_.planPeriod;
        }
        return cmd;
    }

    // ---- driving phases ----
    cmd.active = true;
    if (time_ - startTime_ > config_.maxDuration) {
        abort("timed out");
        cmd.active = false;
        return cmd;
    }
    const double maxSteer = p.maxSteerAngle;
    auto speedCmd = [&](double target, Gear wanted) {
        // Gear can only change at standstill: brake to a stop first.
        if (wanted != gear) target = 0.0;
        const Pedals pd = pedalsForSpeed(target, s.speed, gear, p, maxBrake);
        cmd.throttle = pd.throttle;
        cmd.brake = pd.brake;
        cmd.gear = wanted;
    };
    // Speed that allows stopping `margin` short of the target, never below a crawl so the target is reached.
    auto distanceProfile = [&](double distance, double vmax, double margin) {
        const double d = std::max(distance - margin, 0.0);
        return std::clamp(std::sqrt(2.0 * config_.comfortDecel * d), config_.creepSpeed, vmax);
    };
    const bool stopped = std::abs(s.speed) < 0.05;

    switch (phase_) {
    case ParkingPhase::Align: {
        const double dy = plan_.y0 - s.y;
        const double dir = dy >= 0.0 ? 1.0 : -1.0;
        const Gear wanted = dir > 0 ? Gear::Drive : Gear::Reverse;
        // Wheel-straight hold on the lane line x = x0.
        const double ex = s.x - plan_.x0;
        const double psiDes = (s.speed >= 0 ? -1.0 : 1.0) * std::clamp(0.5 * ex, -0.15, 0.15);
        cmd.steer = headingHold(psiDes, s, p);
        if (std::abs(dy) < config_.alignTol) {
            speedCmd(0.0, gear);
            if (stopped) enter(ParkingPhase::PreSteer, "");
        } else {
            speedCmd(dir * distanceProfile(std::abs(dy) - config_.alignTol, config_.alignSpeed, 0.4), wanted);
            // Look ahead along the road in the direction of travel.
            std::vector<ParkingPose> line;
            for (double d = 0.0; d <= config_.lookahead; d += 0.25) line.push_back({s.x, s.y + dir * d, 0.0});
            const std::size_t saved = progress_;
            progress_ = 0;
            const bool blocked = pathBlocked(line, s, p, sensors);
            progress_ = saved;
            if (blocked) { abort("obstacle on the road"); cmd.active = false; }
        }
        break;
    }
    case ParkingPhase::PreSteer: {
        speedCmd(0.0, Gear::Reverse);
        cmd.steer = plan_.steerAngle / maxSteer;
        if (stopped && gear == Gear::Reverse && std::abs(s.steerAngle - plan_.steerAngle) < 0.02)
            enter(ParkingPhase::Arc, "Parking: reversing into the bay");
        break;
    }
    case ParkingPhase::Arc: {
        const double remaining = s.heading - (-kHalfPi);
        cmd.steer = plan_.steerAngle / maxSteer;
        if (remaining <= 0.005) {
            speedCmd(0.0, Gear::Reverse);
            if (stopped) enter(ParkingPhase::Straighten, "");
        } else {
            speedCmd(-distanceProfile(plan_.radius * remaining, config_.arcSpeed, 0.05), Gear::Reverse);
            if (pathBlocked(plan_.path, s, p, sensors)) { abort("obstacle in the path"); cmd.active = false; }
        }
        break;
    }
    case ParkingPhase::Straighten: {
        speedCmd(0.0, Gear::Reverse);
        cmd.steer = 0.0;
        if (stopped && std::abs(s.steerAngle) < 0.02) enter(ParkingPhase::Straight, "");
        break;
    }
    case ParkingPhase::Straight: {
        const double dx = plan_.xf - s.x;
        if (dx < 0.04) {
            speedCmd(0.0, Gear::Reverse);
            cmd.steer = 0.0;
            if (stopped) enter(ParkingPhase::Settle, "");
        } else {
            const double ey = s.y - plan_.yf;
            const double psiDes = -kHalfPi + std::clamp(0.4 * ey, -0.2, 0.2);
            cmd.steer = headingHold(psiDes, s, p);
            speedCmd(-distanceProfile(dx, config_.straightSpeed, 0.05), Gear::Reverse);
            if (pathBlocked(plan_.path, s, p, sensors)) { abort("obstacle in the path"); cmd.active = false; }
        }
        break;
    }
    case ParkingPhase::Settle: {
        speedCmd(0.0, Gear::Reverse);
        cmd.steer = 0.0;
        SlotReading slot;
        slot.id = plan_.slotId;
        slot.x = plan_.xf;
        slot.y = plan_.yf;
        slot.width = 3.0;
        slot.length = 5.5;
        for (const SlotReading& r : sensors.bays)
            if (r.id == plan_.slotId) slot = r;
        const double headingErr = std::abs(s.heading + kHalfPi);
        const bool inside = insideSlot(s, p, slot, config_.slotTolerance);
        if (inside && headingErr < config_.finalHeadingTol) {
            note(Severity::Info, format("Parked in bay %.0f (offset %.2f m, heading error %.1f deg)", plan_.slotId,
                                        std::hypot(s.x - slot.x, s.y - slot.y), headingErr * 180.0 / kPi));
            phase_ = ParkingPhase::Parked;
            cmd.active = false;
        } else {
            abort(inside ? "finished with a large heading error" : "finished outside the bay");
            cmd.active = false;
        }
        break;
    }
    default: break;
    }
    return cmd;
}

}  // namespace driveforge
