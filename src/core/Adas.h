#pragma once

#include <string>
#include <vector>

#include "Aeb.h"
#include "Acc.h"
#include "EventDetector.h"
#include "LaneKeeping.h"
#include "Overtaking.h"
#include "Parking.h"
#include "VehicleModel.h"

namespace driveforge {

enum class DriveMode { Manual, Autonomous };

enum class AdasRequest {
    ToggleAeb,
    ToggleAcc,
    ToggleLka,
    ToggleAutonomous,
    LaneChangeLeft,
    LaneChangeRight,
    AccSpeedUp,
    AccSpeedDown,
    AccNextTimeGap,
    ToggleParkingAssist,
    StartParking,
    ToggleOvertake,
};

struct AdasConfig {
    AebConfig aeb;
    ParkingConfig parking;
    AccConfig acc;
    LkaConfig lka;
    OvertakeConfig overtake;
    Road road;
    double autonomousSetSpeed = 80.0 / 3.6;  // m/s
    double accMinSetSpeed = 30.0 / 3.6;
    double accMaxSetSpeed = 130.0 / 3.6;
    double accSpeedStep = 5.0 / 3.6;
    std::vector<double> timeGaps = {1.2, 1.8, 2.4};
};

struct AdasStatus {
    DriveMode mode = DriveMode::Manual;
    bool aebArmed = true;
    bool aebWarning = false;
    bool aebBraking = false;
    int aebTargetId = -1;
    double aebTtc = 0.0;
    double aebRequiredDecel = 0.0;

    bool accEnabled = false;
    double accSetSpeed = 0.0;  // m/s
    double accTimeGap = 1.8;
    bool accHasLead = false;
    double accLeadGap = 0.0;

    bool lkaEnabled = false;
    bool lkaOverridden = false;
    int lkaTargetLane = 0;
    bool laneChanging = false;
    int laneChangeDir = 0;  // -1 left, +1 right while changing lanes
    double lkaOffset = 0.0;

    bool overtakeEnabled = true;  // autonomous overtaking / obstacle avoidance allowed in Autonomous mode
    OvertakePhase overtakePhase = OvertakePhase::Idle;
    std::string overtakeText;     // what the autonomous planner is doing, for the dashboard

    bool parkingEnabled = false;
    bool parkingActive = false;  // the assist is driving the car
    ParkingPhase parkingPhase = ParkingPhase::Off;
    int parkingSlot = -1;
};

// Combines driver input with AEB, ACC and lane keeping. ADAS only ever sees sensor readings and returns pedal/steer
// commands; it never touches vehicle state. Commands from the UI are queued as requests and handled in update().
class Adas {
public:
    explicit Adas(AdasConfig config = {});

    void reset();
    void request(AdasRequest request) { pending_.push_back(request); }
    void disengageAll(const std::string& reason);
    void releaseAll();  // silent: scenario ended, hand control back
    // Autonomous mode (and ACC) will use this set speed / time gap when engaged.
    void setAutonomousSetpoint(double speed, double timeGap);

    ControlInput update(const ControlInput& driver, const VehicleModel& vehicle, const SensorReadings& sensors,
                        double dt);

    const AdasStatus& status() const { return status_; }
    const ParkingAssist& parking() const { return parking_; }
    std::vector<Event> takeEvents();

private:
    void handle(AdasRequest request, const ControlInput& driver, const VehicleModel& vehicle,
                const SensorReadings& sensors);
    void engageAcc(const VehicleState& state);
    // Starts parking if a bay is ready; returns true once the request is settled (started or refused).
    bool tryStartParking(const VehicleModel& vehicle);
    void note(Severity severity, const std::string& message);

    AdasConfig config_;
    Aeb aeb_;
    ParkingAssist parking_;
    LaneKeeping lka_;
    Overtaking overtake_;
    AdasStatus status_;
    std::vector<AdasRequest> pending_;
    std::vector<Event> events_;
    double time_ = 0.0;
    bool takeoverArmed_ = false;  // driver controls must be released once after parking starts before they count as a takeover
    bool startPending_ = false;  // Y was pressed: start as soon as a bay is ready
    double startPendingUntil_ = 0.0;
    int timeGapIndex_ = 1;
    bool prevAebWarning_ = false;
    bool prevAebBraking_ = false;
};

}  // namespace driveforge
