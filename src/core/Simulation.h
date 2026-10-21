#pragma once

#include <set>
#include <vector>

#include "Adas.h"
#include "EventDetector.h"
#include "Sensors.h"
#include "Telemetry.h"
#include "VehicleModel.h"
#include "World.h"

namespace driveforge {

// Owns the whole simulation and its step order (see docs/DESIGN.md):
// world -> sensors -> ADAS -> vehicle -> collisions/events/telemetry. Qt-free so tests run the real loop.
class Simulation {
public:
    Simulation();

    void reset();
    void step(double dt);

    ControlInput& driver() { return driver_; }
    const ControlInput& driver() const { return driver_; }
    VehicleModel& vehicle() { return vehicle_; }
    const VehicleModel& vehicle() const { return vehicle_; }
    World& world() { return world_; }
    const World& world() const { return world_; }
    Adas& adas() { return adas_; }
    const Adas& adas() const { return adas_; }
    const SensorReadings& sensors() const { return sensors_; }
    const Telemetry& telemetry() const { return telemetry_; }
    int collisions() const { return collisions_; }
    // Gap to the nearest object ahead in the ego lane corridor, or the sensor range if none.
    double frontGap() const { return frontGap_; }

    // Spawn helpers (relative to the ego car, in the ego's current lane); return the new object id.
    int spawnObstacle(double ahead);
    int spawnPedestrian(double ahead, double triggerTtc = 3.5);
    int spawnLeadVehicle(double ahead, double speed);
    // Replaces any parking lot with a row of 8 bays whose first free bay (bay 4) is `ahead` m in front of the car.
    // Bays 4 and 7 are free, the rest hold parked cars.
    void spawnParkingLot(double ahead);

    std::vector<Event> takeEvents();
    bool takeTelemetryFlag();  // true if a telemetry sample was recorded since the last call

private:
    VehicleModel vehicle_;
    World world_;
    Adas adas_;
    ControlInput driver_;
    SensorReadings sensors_;
    Telemetry telemetry_;
    EventDetector detector_;
    std::vector<Event> events_;
    std::set<int> touching_;
    int collisions_ = 0;
    double frontGap_ = 150.0;
    bool sampled_ = false;
};

}  // namespace driveforge
