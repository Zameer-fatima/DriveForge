#include "Simulation.h"

#include <algorithm>
#include <cmath>

namespace driveforge {

Simulation::Simulation() { reset(); }

void Simulation::reset() {
    vehicle_.reset();
    world_.clear();
    world_.clearBays();
    adas_.reset();
    driver_ = ControlInput{};
    sensors_ = SensorReadings{};
    telemetry_.clear();
    detector_.reset(vehicle_.state(), vehicle_.ignition(), vehicle_.gear());
    events_.clear();
    touching_.clear();
    collisions_ = 0;
    frontGap_ = 150.0;
    sampled_ = false;
}

void Simulation::step(double dt) {
    const VehicleState& s = vehicle_.state();
    world_.step(dt, s);
    sensors_ = sense(world_, s, vehicle_.params());

    vehicle_.setInput(adas_.update(driver_, vehicle_, sensors_, dt));
    vehicle_.step(dt);

    // Collisions: stop dead on first contact with each object.
    for (const WorldObject& o : world_.objects()) {
        if (overlaps(vehicle_.state(), vehicle_.params(), o)) {
            if (touching_.insert(o.id).second) {
                ++collisions_;
                vehicle_.forceSpeed(0.0);
                events_.push_back({vehicle_.state().time, Severity::Warning, std::string("COLLISION with ") + objectName(o.type)});
                adas_.disengageAll("collision");
            }
        } else {
            touching_.erase(o.id);
        }
    }

    const Detection* front = nearestInCorridor(sensors_, world_.road().laneWidth / 2.0);
    frontGap_ = front ? front->gap : sensors_.frontRange;

    for (Event& e : detector_.update(vehicle_.state(), vehicle_.ignition(), vehicle_.gear())) events_.push_back(std::move(e));
    for (Event& e : adas_.takeEvents()) events_.push_back(std::move(e));
    sampled_ |= telemetry_.update(vehicle_.state(), frontGap_);
}

int Simulation::spawnObstacle(double ahead) {
    const VehicleState& s = vehicle_.state();
    const double x = world_.road().laneCenter(world_.road().laneIndexAt(s.x));
    return world_.add(World::makeObstacle(x, s.y + ahead));
}

int Simulation::spawnPedestrian(double ahead, double triggerTtc) {
    const VehicleState& s = vehicle_.state();
    const Road& road = world_.road();
    // Starts on the right kerb and walks left across all lanes.
    return world_.add(World::makePedestrian(road.halfWidth() + 1.5, s.y + ahead, -1.4, triggerTtc));
}

int Simulation::spawnLeadVehicle(double ahead, double speed) {
    const VehicleState& s = vehicle_.state();
    const double x = world_.road().laneCenter(world_.road().laneIndexAt(s.x));
    return world_.add(World::makeVehicle(x, s.y + ahead, speed));
}

void Simulation::spawnParkingLot(double ahead) {
    world_.clearLot();
    const double bayPitch = ParkingSlot{}.width;
    world_.addParkingRow(vehicle_.state().y + ahead - 3.0 * bayPitch, 8, {true, true, true, false, true, true, false, true});
}

std::vector<Event> Simulation::takeEvents() {
    std::vector<Event> out;
    out.swap(events_);
    return out;
}

bool Simulation::takeTelemetryFlag() {
    const bool v = sampled_;
    sampled_ = false;
    return v;
}

}  // namespace driveforge
