#include "Sensors.h"

#include <algorithm>
#include <cmath>

namespace driveforge {

SensorReadings sense(const World& world, const VehicleState& ego, const VehicleParams& p, const SensorConfig& cfg) {
    SensorReadings out;
    out.frontRange = cfg.frontRange;

    const double ux = std::sin(ego.heading), uy = std::cos(ego.heading);
    const double rx = uy, ry = -ux;
    const double evx = ego.speed * ux, evy = ego.speed * uy;

    for (const WorldObject& o : world.objects()) {
        const double dx = o.x - ego.x, dy = o.y - ego.y;
        const double f = dx * ux + dy * uy;
        const double l = dx * rx + dy * ry;
        if (std::abs(l) > cfg.sideRange) continue;

        // Edge-to-edge separation along the heading (length of the object projected on the heading is approximated by o.length).
        const double reach = p.length / 2.0 + o.length / 2.0;
        double gap = 0.0;
        if (f > reach) gap = f - reach;
        else if (f < -reach) gap = f + reach;
        if (gap > cfg.frontRange || gap < -cfg.rearRange) continue;

        const double ovx = o.active ? o.vx : 0.0, ovy = o.active ? o.vy : 0.0;
        Detection d;
        d.id = o.id;
        d.type = o.type;
        d.gap = gap;
        d.lateral = l;
        d.worldX = o.x;
        d.worldY = o.y;
        d.halfWidth = o.width / 2.0;
        d.halfLength = o.length / 2.0;
        d.closingSpeed = (evx - ovx) * ux + (evy - ovy) * uy;
        d.lateralSpeed = (ovx - evx) * rx + (ovy - evy) * ry;
        d.objectSpeed = ovx * ux + ovy * uy;
        out.detections.push_back(d);
    }
    std::sort(out.detections.begin(), out.detections.end(),
              [](const Detection& a, const Detection& b) { return a.gap < b.gap; });

    for (const ParkingSlot& slot : world.bays()) {
        if (std::abs(slot.y - ego.y) > cfg.slotRange) continue;
        SlotReading r;
        r.id = slot.id;
        r.x = slot.x;
        r.y = slot.y;
        r.width = slot.width;
        r.length = slot.length;
        r.free = true;
        for (const WorldObject& o : world.objects()) {
            if (std::abs(o.x - slot.x) < (o.width + slot.length) / 2.0 - 0.1 &&
                std::abs(o.y - slot.y) < (o.length + slot.width) / 2.0 - 0.1) {
                r.free = false;
                break;
            }
        }
        out.bays.push_back(r);
    }
    std::sort(out.bays.begin(), out.bays.end(), [&](const SlotReading& a, const SlotReading& b) {
        return std::abs(a.y - ego.y) < std::abs(b.y - ego.y);
    });

    const Road& road = world.road();
    out.cameraOk = road.laneVisible(ego.x);
    out.lane.valid = out.cameraOk;
    if (out.cameraOk) {
        out.lane.laneIndex = road.laneIndexAt(ego.x);
        out.lane.offset = ego.x - road.laneCenter(out.lane.laneIndex);
        out.lane.headingError = ego.heading;
    }
    return out;
}

const Detection* nearestInCorridor(const SensorReadings& readings, double halfCorridor) {
    for (const Detection& d : readings.detections)
        if (d.gap >= 0.0 && std::abs(d.lateral) < halfCorridor + d.halfWidth) return &d;
    return nullptr;
}

}  // namespace driveforge
