#pragma once

#include <vector>

#include "Road.h"
#include "VehicleModel.h"

namespace driveforge {

enum class ObjectType { Vehicle, Obstacle, Pedestrian };

// Axis-aligned box on the road plane (x right, y forward). Simulated, not measured.
struct WorldObject {
    int id = 0;
    ObjectType type = ObjectType::Obstacle;
    double x = 0.0, y = 0.0;  // centre, m
    double width = 1.0;       // along x
    double length = 1.0;      // along y
    double vx = 0.0, vy = 0.0;
    // If >= 0 the object stands still until the ego car is within this many seconds of reaching it.
    double triggerTtc = -1.0;
    bool active = true;
    bool persistent = false;  // parked cars are never culled when far behind the ego car
};

// Perpendicular parking bay to the right of the road. Opens towards the road (-x). Centre (x, y).
struct ParkingSlot {
    int id = 0;
    double x = 0.0, y = 0.0;
    double width = 3.0;   // extent along y
    double length = 5.5;  // extent along x
};

class World {
public:
    explicit World(Road road = {}) : road_(road) {}

    const Road& road() const { return road_; }
    int add(WorldObject object);
    void clear();
    void step(double dt, const VehicleState& ego);

    const std::vector<WorldObject>& objects() const { return objects_; }
    WorldObject* find(int id);  // for scenario scripts that steer other actors
    // Changes whenever objects are added or removed (not when they merely move).
    int version() const { return version_; }

    // A row of `count` bays starting at startY (first bay centre), beside the road. occupied[i] puts a parked car in bay i.
    // Returns the id of the first bay.
    int addParkingRow(double startY, int count, const std::vector<bool>& occupied);
    void clearBays();
    // Removes the bays and the parked cars in them.
    void clearLot();
    const std::vector<ParkingSlot>& bays() const { return bays_; }
    int baysVersion() const { return baysVersion_; }

    static WorldObject makeVehicle(double x, double y, double speed);
    static WorldObject makeObstacle(double x, double y);
    static WorldObject makePedestrian(double x, double y, double vx, double triggerTtc);
    static WorldObject makeParkedCar(double x, double y);  // across the bay: 4.5 m along x, 1.8 m along y

private:
    Road road_;
    std::vector<WorldObject> objects_;
    std::vector<ParkingSlot> bays_;
    int baysVersion_ = 0;
    int nextId_ = 1;
    int version_ = 0;
};

// Oriented ego rectangle vs axis-aligned object box (separating axis test).
bool overlaps(const VehicleState& ego, const VehicleParams& params, const WorldObject& object);

// Smallest distance between the ego rectangle and the object box (0 when touching). Ground truth, for scoring only.
double separation(const VehicleState& ego, const VehicleParams& params, const WorldObject& object);

const char* objectName(ObjectType type);

}  // namespace driveforge
