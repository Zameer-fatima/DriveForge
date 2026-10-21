#pragma once

#include <vector>

#include "World.h"

namespace driveforge {

// What the simulated forward radar reports for one object, in the ego frame. Ideal (noise-free) and SIMULATED.
struct Detection {
    int id = 0;
    ObjectType type = ObjectType::Obstacle;
    double gap = 0.0;           // m, bumper-to-bumper along ego heading; > 0 ahead, < 0 behind, 0 overlapping
    double lateral = 0.0;       // m, object centre relative to ego, positive = right
    double worldX = 0.0;        // m, object centre (used to assign road lanes)
    double worldY = 0.0;
    double halfWidth = 0.0;     // m, half extent along x
    double halfLength = 0.0;    // m, half extent along y
    double closingSpeed = 0.0;  // m/s, positive = gap is shrinking
    double lateralSpeed = 0.0;  // m/s, object velocity across ego heading, positive = towards the right
    double objectSpeed = 0.0;   // m/s along ego heading
};

struct LaneReading {
    bool valid = false;         // markings visible
    int laneIndex = 0;
    double offset = 0.0;        // m, ego centre minus lane centre, positive = right of centre
    double headingError = 0.0;  // rad, positive = pointing right of the lane direction
};

// A parking bay seen by the simulated side sensors.
struct SlotReading {
    int id = 0;
    double x = 0.0, y = 0.0, width = 0.0, length = 0.0;
    bool free = false;  // no object overlaps the bay
};

struct SensorConfig {
    double frontRange = 150.0;  // m
    double rearRange = 20.0;    // m
    double sideRange = 16.0;    // m, max |lateral|
    double slotRange = 40.0;    // m, along the road
};

struct SensorReadings {
    std::vector<Detection> detections;  // sorted by gap, ascending (rear objects first)
    LaneReading lane;
    std::vector<SlotReading> bays;  // bays within slotRange, nearest first
    double frontRange = 150.0;
    bool radarOk = true;
    bool cameraOk = false;
};

SensorReadings sense(const World& world, const VehicleState& ego, const VehicleParams& params,
                     const SensorConfig& config = {});

// Nearest object ahead whose lateral position is within |lateral| < halfCorridor, or nullptr.
const Detection* nearestInCorridor(const SensorReadings& readings, double halfCorridor);

}  // namespace driveforge
