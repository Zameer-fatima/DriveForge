#include "Adas.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace driveforge {

namespace {
std::string kmh(double ms) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.0f km/h", ms * 3.6);
    return buf;
}
}  // namespace

Adas::Adas(AdasConfig config)
    : config_(std::move(config)), aeb_(config_.aeb), parking_(config_.parking, config_.road), lka_(config_.lka, config_.road) {
    reset();
}

void Adas::reset() {
    aeb_.reset();
    parking_.reset();
    lka_.disengage();
    const bool armed = true;
    status_ = AdasStatus{};
    status_.aebArmed = armed;
    status_.accTimeGap = config_.timeGaps[timeGapIndex_ = std::min<int>(1, static_cast<int>(config_.timeGaps.size()) - 1)];
    pending_.clear();
    events_.clear();
    startPending_ = false;
    takeoverArmed_ = false;
    time_ = 0.0;
    prevAebWarning_ = prevAebBraking_ = false;
}

std::vector<Event> Adas::takeEvents() {
    std::vector<Event> out;
    out.swap(events_);
    return out;
}

void Adas::note(Severity severity, const std::string& message) { events_.push_back({time_, severity, message}); }

void Adas::disengageAll(const std::string& reason) {
    parking_.abort(reason);
    const bool any = status_.accEnabled || status_.lkaEnabled || status_.mode == DriveMode::Autonomous;
    status_.accEnabled = false;
    status_.lkaEnabled = false;
    status_.lkaOverridden = false;
    status_.mode = DriveMode::Manual;
    lka_.disengage();
    if (any) note(Severity::Warning, "ADAS disengaged: " + reason);
}

void Adas::releaseAll() {
    status_.accEnabled = false;
    status_.lkaEnabled = false;
    status_.lkaOverridden = false;
    status_.mode = DriveMode::Manual;
    lka_.disengage();
    if (parking_.enabled()) {
        parking_.abort("scenario ended");
        parking_.setEnabled(false);
        parking_.takeEvents();
    }
    startPending_ = false;
}

void Adas::setAutonomousSetpoint(double speed, double timeGap) {
    config_.autonomousSetSpeed = speed;
    status_.accTimeGap = timeGap;
    if (status_.mode == DriveMode::Autonomous) status_.accSetSpeed = speed;
}

void Adas::engageAcc(const VehicleState& s) {
    const double rounded = std::round(s.speed / config_.accSpeedStep) * config_.accSpeedStep;
    status_.accSetSpeed = std::clamp(rounded, config_.accMinSetSpeed, config_.accMaxSetSpeed);
    status_.accEnabled = true;
    note(Severity::Info, "ACC ON, set speed " + kmh(status_.accSetSpeed));
}

bool Adas::tryStartParking(const VehicleModel& vehicle) {
    const VehicleState& s = vehicle.state();
    if (parking_.phase() != ParkingPhase::Ready) {
        if (s.time <= startPendingUntil_) return false;  // keep searching
        note(Severity::Warning, "Parking cannot start: no usable bay found (" + parking_.statusText() + ")");
        return true;
    }
    std::string why;
    if (!parking_.start(s, vehicle.params(), &why)) {
        note(Severity::Warning, "Parking cannot start: " + why);
        return true;
    }
    if (status_.accEnabled || status_.lkaEnabled || status_.mode == DriveMode::Autonomous) {
        status_.accEnabled = false;
        status_.lkaEnabled = false;
        status_.lkaOverridden = false;
        status_.mode = DriveMode::Manual;
        lka_.disengage();
        note(Severity::Info, "ACC / lane keeping / autonomous off for parking");
    }
    return true;
}

void Adas::handle(AdasRequest r, const ControlInput& driver, const VehicleModel& vehicle, const SensorReadings& sensors) {
    const VehicleState& s = vehicle.state();
    const bool canEngage = driver.ignition && vehicle.gear() == Gear::Drive;
    auto refuse = [&](const std::string& why) { note(Severity::Warning, why); };

    const bool parkingBusy = parking_.active();
    if (parkingBusy && (r == AdasRequest::ToggleAcc || r == AdasRequest::ToggleLka || r == AdasRequest::ToggleAutonomous ||
                        r == AdasRequest::LaneChangeLeft || r == AdasRequest::LaneChangeRight)) {
        refuse("Parking in progress: finish or cancel it first (N)");
        return;
    }

    switch (r) {
    case AdasRequest::ToggleParkingAssist:
        if (parking_.enabled()) parking_.abort("assist switched off"), parking_.setEnabled(false);
        else if (!driver.ignition) refuse("Parking assist unavailable: ignition off");
        else if (!sensors.cameraOk) refuse("Parking assist unavailable: not on the road");
        else parking_.setEnabled(true);
        break;
    case AdasRequest::StartParking:
        if (parking_.active()) { note(Severity::Info, "Parking already in progress"); break; }
        if (!driver.ignition) { refuse("Parking unavailable: ignition off"); break; }
        if (!parking_.enabled()) {
            if (!sensors.cameraOk) { refuse("Parking assist unavailable: not on the road"); break; }
            parking_.setEnabled(true);
        }
        // The bay search runs in update(); start as soon as a plan is ready (waits up to 10 s).
        startPending_ = true;
        startPendingUntil_ = s.time + 10.0;
        break;
    case AdasRequest::ToggleAeb:
        status_.aebArmed = !status_.aebArmed;
        if (!status_.aebArmed) aeb_.reset();
        note(status_.aebArmed ? Severity::Info : Severity::Warning, status_.aebArmed ? "AEB armed" : "AEB DISABLED by driver");
        break;
    case AdasRequest::ToggleAcc:
        if (status_.mode == DriveMode::Autonomous) { refuse("ACC is part of Autonomous mode; leave it first (M)"); break; }
        if (status_.accEnabled) { status_.accEnabled = false; note(Severity::Info, "ACC OFF"); }
        else if (!canEngage) refuse("ACC unavailable: ignition off or not in Drive");
        else engageAcc(s);
        break;
    case AdasRequest::ToggleLka:
        if (status_.mode == DriveMode::Autonomous) { refuse("Lane keeping is part of Autonomous mode; leave it first (M)"); break; }
        if (status_.lkaEnabled) {
            status_.lkaEnabled = false;
            status_.lkaOverridden = false;
            lka_.disengage();
            note(Severity::Info, "Lane keeping OFF");
        } else if (!canEngage) refuse("Lane keeping unavailable: ignition off or not in Drive");
        else if (!sensors.cameraOk) refuse("Lane keeping unavailable: lane markings not visible");
        else { lka_.engage(s); status_.lkaEnabled = true; note(Severity::Info, "Lane keeping ON"); }
        break;
    case AdasRequest::ToggleAutonomous:
        if (status_.mode == DriveMode::Autonomous) {
            status_.mode = DriveMode::Manual;
            status_.accEnabled = false;
            status_.lkaEnabled = false;
            lka_.disengage();
            note(Severity::Info, "Autonomous mode OFF, manual control");
        } else if (!canEngage) refuse("Autonomous mode unavailable: ignition off or not in Drive");
        else if (!sensors.cameraOk) refuse("Autonomous mode unavailable: lane markings not visible");
        else {
            lka_.engage(s);
            status_.lkaEnabled = true;
            status_.accEnabled = true;
            status_.aebArmed = true;
            status_.accSetSpeed = config_.autonomousSetSpeed;
            status_.mode = DriveMode::Autonomous;
            note(Severity::Info, "Autonomous mode ON, set speed " + kmh(status_.accSetSpeed));
        }
        break;
    case AdasRequest::LaneChangeLeft:
    case AdasRequest::LaneChangeRight: {
        if (!status_.lkaEnabled) { refuse("Lane change needs Lane keeping (K) or Autonomous mode (M)"); break; }
        if (status_.lkaOverridden) { refuse("Lane change refused: driver is steering"); break; }
        const int dir = r == AdasRequest::LaneChangeLeft ? -1 : 1;
        std::string why;
        if (lka_.startLaneChange(dir, s, sensors, &why))
            note(Severity::Info, std::string("Lane change ") + (dir < 0 ? "left" : "right") + " started");
        else
            refuse("Lane change refused: " + why);
        break;
    }
    case AdasRequest::AccSpeedUp:
    case AdasRequest::AccSpeedDown:
        if (!status_.accEnabled) break;
        status_.accSetSpeed = std::clamp(status_.accSetSpeed + (r == AdasRequest::AccSpeedUp ? 1 : -1) * config_.accSpeedStep,
                                         config_.accMinSetSpeed, config_.accMaxSetSpeed);
        note(Severity::Info, "ACC set speed " + kmh(status_.accSetSpeed));
        break;
    case AdasRequest::AccNextTimeGap: {
        timeGapIndex_ = (timeGapIndex_ + 1) % static_cast<int>(config_.timeGaps.size());
        status_.accTimeGap = config_.timeGaps[timeGapIndex_];
        char buf[48];
        std::snprintf(buf, sizeof buf, "ACC time gap %.1f s", status_.accTimeGap);
        note(Severity::Info, buf);
        break;
    }
    }
}

ControlInput Adas::update(const ControlInput& driver, const VehicleModel& vehicle, const SensorReadings& sensors, double dt) {
    const VehicleState& s = vehicle.state();
    const VehicleParams& p = vehicle.params();
    time_ = s.time;

    parking_.setTime(s.time);
    for (AdasRequest r : pending_) handle(r, driver, vehicle, sensors);
    pending_.clear();

    // Automatic cancel conditions.
    if (!driver.ignition) {
        disengageAll("ignition off");
        if (parking_.enabled()) { parking_.abort("ignition off"); parking_.setEnabled(false); }
    }
    if (parking_.active()) {
        // Keys that were already held when parking started must not cancel it: wait for one release first.
        const bool driverActing = driver.brake > 0.05 || std::abs(driver.steer) > 0.05 || driver.throttle > 0.05;
        if (!driverActing) takeoverArmed_ = true;
        else if (takeoverArmed_) parking_.abort("driver takeover");
    } else {
        takeoverArmed_ = false;
    }
    if (vehicle.gear() == Gear::Reverse && (status_.accEnabled || status_.lkaEnabled)) disengageAll("reverse gear");
    if (status_.lkaEnabled && !sensors.cameraOk) disengageAll("lane markings lost");

    const bool driverBraking = driver.brake > 0.05;
    const bool driverSteering = std::abs(driver.steer) > 0.05;
    if (status_.mode == DriveMode::Autonomous && (driverBraking || driverSteering)) {
        status_.mode = DriveMode::Manual;
        status_.accEnabled = false;
        status_.lkaEnabled = false;
        status_.lkaOverridden = false;
        lka_.disengage();
        note(Severity::Warning, "Driver takeover: Autonomous mode OFF");
    } else if (status_.accEnabled && driverBraking && status_.mode == DriveMode::Manual) {
        status_.accEnabled = false;
        note(Severity::Info, "ACC cancelled by driver braking");
    }

    // Lane keeping yields to the driver's steering and resumes (re-centring in the nearest lane) afterwards.
    if (status_.lkaEnabled) {
        if (driverSteering && !status_.lkaOverridden) {
            status_.lkaOverridden = true;
            note(Severity::Info, "Lane keeping suspended: driver steering");
        } else if (!driverSteering && status_.lkaOverridden) {
            status_.lkaOverridden = false;
            lka_.engage(s);
            note(Severity::Info, "Lane keeping resumed");
        }
    }

    ControlInput out = driver;
    const bool autonomous = status_.mode == DriveMode::Autonomous;
    if (autonomous) {
        out.throttle = out.brake = out.steer = 0.0;
    }

    // ACC
    status_.accHasLead = false;
    if (status_.accEnabled) {
        const AccOutput acc = computeAcc(config_.acc, status_.accSetSpeed, status_.accTimeGap, s, p, sensors,
                                         vehicle.maxBrakeDeceleration());
        status_.accHasLead = acc.hasLead;
        status_.accLeadGap = acc.gap;
        out.throttle = std::max(out.throttle, acc.throttle);
        if (acc.brake > 0.0 && driver.throttle < 0.05) out.brake = std::max(out.brake, acc.brake);
        if (driver.throttle >= 0.05 && !autonomous) out.brake = driver.brake;
    }

    // Lane keeping
    status_.laneChanging = lka_.laneChanging();
    if (status_.lkaEnabled && !status_.lkaOverridden) {
        const LkaOutput lka = lka_.update(s, p, dt);
        out.steer = lka.steer;
        status_.lkaOffset = lka.offset;
        status_.laneChanging = lka_.laneChanging();
        if (lka.laneChangeCompleted) note(Severity::Info, "Lane change complete");
    }
    status_.laneChangeDir = lka_.laneChangeDirection();
    status_.lkaTargetLane = lka_.targetLane();

    // Parking assist: while it is driving, it replaces the driver's pedals, steering and gear.
    if (parking_.enabled()) {
        const ParkingCommand park = parking_.update(s, p, vehicle.gear(), sensors, vehicle.maxBrakeDeceleration(), dt);
        if (park.active) {
            out.throttle = park.throttle;
            out.brake = park.brake;
            out.steer = park.steer;
            out.gear = park.gear;
        }
    }
    if (startPending_ && tryStartParking(vehicle)) startPending_ = false;
    if (!parking_.enabled()) startPending_ = false;
    status_.parkingEnabled = parking_.enabled();
    status_.parkingActive = parking_.active();
    status_.parkingPhase = parking_.phase();
    status_.parkingSlot = parking_.slotId();
    for (Event& e : parking_.takeEvents()) events_.push_back(std::move(e));

    // AEB has the final say on pedals.
    AebOutput aeb;
    if (status_.aebArmed) aeb = aeb_.update(s, p, sensors, vehicle.maxBrakeDeceleration(), out.throttle, dt);
    status_.aebWarning = aeb.warning;
    status_.aebBraking = aeb.braking;
    status_.aebTargetId = aeb.targetId;
    status_.aebTtc = aeb.ttc;
    status_.aebRequiredDecel = aeb.requiredDecel;
    if (aeb.inhibitThrottle) out.throttle = 0.0;
    if (aeb.braking) out.brake = std::max(out.brake, aeb.brake);
    if (aeb.braking && !prevAebBraking_) {
        char buf[96];
        std::snprintf(buf, sizeof buf, "AEB: emergency braking (TTC %.1f s)", aeb.ttc);
        note(Severity::Warning, buf);
    } else if (!aeb.braking && prevAebBraking_) {
        note(Severity::Info, "AEB released");
    } else if (aeb.warning && !prevAebWarning_ && !aeb.braking) {
        note(Severity::Warning, "AEB: collision warning");
    }
    prevAebWarning_ = aeb.warning;
    prevAebBraking_ = aeb.braking;
    return out;
}

}  // namespace driveforge
