#include "World.h"

#include <algorithm>
#include <cmath>

namespace driveforge {

int World::add(WorldObject object) {
    object.id = nextId_++;
    object.active = object.triggerTtc < 0.0;
    objects_.push_back(object);
    ++version_;
    return object.id;
}

void World::clear() {
    if (!objects_.empty()) ++version_;
    objects_.clear();
}

void World::step(double dt, const VehicleState& ego) {
    for (WorldObject& o : objects_) {
        if (!o.active) {
            const double ahead = o.y - ego.y;
            const double speed = std::max(ego.speed, 0.5);
            if (ahead <= 0.0 || ahead / speed <= o.triggerTtc) o.active = true;
        }
        if (o.active) {
            o.x += o.vx * dt;
            o.y += o.vy * dt;
        }
    }
    const auto gone = [&](const WorldObject& o) {
        return !o.persistent && (o.y < ego.y - 150.0 || o.y > ego.y + 800.0);
    };
    const auto it = std::remove_if(objects_.begin(), objects_.end(), gone);
    if (it != objects_.end()) {
        objects_.erase(it, objects_.end());
        ++version_;
    }
}

WorldObject World::makeVehicle(double x, double y, double speed) {
    WorldObject o;
    o.type = ObjectType::Vehicle;
    o.x = x;
    o.y = y;
    o.width = 1.8;
    o.length = 4.5;
    o.vy = speed;
    return o;
}

WorldObject World::makeObstacle(double x, double y) {
    WorldObject o;
    o.type = ObjectType::Obstacle;
    o.x = x;
    o.y = y;
    o.width = 1.6;
    o.length = 1.0;
    return o;
}

WorldObject World::makePedestrian(double x, double y, double vx, double triggerTtc) {
    WorldObject o;
    o.type = ObjectType::Pedestrian;
    o.x = x;
    o.y = y;
    o.width = 0.6;
    o.length = 0.6;
    o.vx = vx;
    o.triggerTtc = triggerTtc;
    return o;
}

int World::addParkingRow(double startY, int count, const std::vector<bool>& occupied) {
    const int first = static_cast<int>(bays_.size()) + 1;
    const double edgeX = road_.halfWidth() + 3.0;  // bays start 3 m beyond the road edge (2.5 m shoulder + 0.5 m)
    for (int i = 0; i < count; ++i) {
        ParkingSlot slot;
        slot.id = first + i;
        slot.x = edgeX + slot.length / 2.0;
        slot.y = startY + i * slot.width;
        bays_.push_back(slot);
        if (i < static_cast<int>(occupied.size()) && occupied[static_cast<std::size_t>(i)])
            add(makeParkedCar(slot.x, slot.y));
    }
    ++baysVersion_;
    return first;
}

void World::clearBays() {
    if (!bays_.empty()) ++baysVersion_;
    bays_.clear();
}

void World::clearLot() {
    const auto it = std::remove_if(objects_.begin(), objects_.end(), [](const WorldObject& o) { return o.persistent; });
    if (it != objects_.end()) {
        objects_.erase(it, objects_.end());
        ++version_;
    }
    clearBays();
}

WorldObject World::makeParkedCar(double x, double y) {
    WorldObject o;
    o.type = ObjectType::Vehicle;
    o.x = x;
    o.y = y;
    o.width = 4.5;
    o.length = 1.8;
    o.persistent = true;
    return o;
}

bool overlaps(const VehicleState& ego, const VehicleParams& p, const WorldObject& o) {
    const double ux = std::sin(ego.heading), uy = std::cos(ego.heading);  // forward
    const double rx = uy, ry = -ux;                                       // right
    const double a = p.length / 2.0, b = p.width / 2.0;
    const double hx = o.width / 2.0, hy = o.length / 2.0;
    const double dx = o.x - ego.x, dy = o.y - ego.y;

    const double axes[4][2] = {{1, 0}, {0, 1}, {ux, uy}, {rx, ry}};
    for (const auto& n : axes) {
        const double ra = a * std::abs(ux * n[0] + uy * n[1]) + b * std::abs(rx * n[0] + ry * n[1]);
        const double rb = hx * std::abs(n[0]) + hy * std::abs(n[1]);
        if (std::abs(dx * n[0] + dy * n[1]) > ra + rb) return false;
    }
    return true;
}

WorldObject* World::find(int id) {
    for (WorldObject& o : objects_)
        if (o.id == id) return &o;
    return nullptr;
}

double separation(const VehicleState& ego, const VehicleParams& p, const WorldObject& o) {
    if (overlaps(ego, p, o)) return 0.0;
    const double ux = std::sin(ego.heading), uy = std::cos(ego.heading);
    const double rx = uy, ry = -ux;
    const double a = p.length / 2.0, b = p.width / 2.0;
    const double hx = o.width / 2.0, hy = o.length / 2.0;
    double best = 1e18;

    // Object sample points (corners, edge midpoints, centre) against the ego rectangle.
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            const double dx = o.x + i * hx - ego.x, dy = o.y + j * hy - ego.y;
            const double f = dx * ux + dy * uy, l = dx * rx + dy * ry;
            best = std::min(best, std::hypot(std::max(std::abs(f) - a, 0.0), std::max(std::abs(l) - b, 0.0)));
        }
    }
    // Ego sample points against the object box.
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            const double px = ego.x + i * a * ux + j * b * rx, py = ego.y + i * a * uy + j * b * ry;
            best = std::min(best, std::hypot(std::max(std::abs(px - o.x) - hx, 0.0), std::max(std::abs(py - o.y) - hy, 0.0)));
        }
    }
    return best;
}

const char* objectName(ObjectType type) {
    switch (type) {
    case ObjectType::Vehicle: return "vehicle";
    case ObjectType::Obstacle: return "obstacle";
    case ObjectType::Pedestrian: return "pedestrian";
    }
    return "object";
}

}  // namespace driveforge
