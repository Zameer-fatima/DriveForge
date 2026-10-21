#include <QtTest>
#include <cmath>

#include "Simulation.h"

using namespace driveforge;

namespace {
constexpr double kDt = 1.0 / 120.0;
constexpr double kHalfPi = 1.57079632679489661923;

void run(Simulation& sim, double seconds) {
    const int n = static_cast<int>(seconds / kDt);
    for (int i = 0; i < n; ++i) sim.step(kDt);
}

// Steps until `done` or timeout (simulated seconds). Returns true if `done` became true.
template <typename F>
bool runUntil(Simulation& sim, double timeout, F done) {
    const int n = static_cast<int>(timeout / kDt);
    for (int i = 0; i < n; ++i) {
        sim.step(kDt);
        if (done()) return true;
    }
    return false;
}

const std::vector<bool> kLot = {true, true, true, false, true, true, false, true};

void setup(Simulation& sim, double x, double y, double speed, double lotStartY, double heading = 0.0) {
    sim.vehicle().reset(x, y, heading, speed);
    sim.driver().ignition = true;
    sim.world().addParkingRow(lotStartY, 8, kLot);
}

ParkingPhase phase(const Simulation& s) { return s.adas().status().parkingPhase; }

bool waitReady(Simulation& sim) {
    sim.adas().request(AdasRequest::ToggleParkingAssist);
    return runUntil(sim, 5.0, [&] { return phase(sim) == ParkingPhase::Ready; });
}

bool hasEvent(Simulation& sim, const char* text) {
    static std::vector<Event> all;
    for (Event& e : sim.takeEvents()) all.push_back(e);
    for (const Event& e : all)
        if (e.message.find(text) != std::string::npos) return true;
    return false;
}
void flush(Simulation& sim) { sim.takeEvents(); }

struct Outcome {
    bool parked = false;
    bool inside = false;
    double headingErrorDeg = 0.0;
    int collisions = 0;
    double time = 0.0;
};

// Full scenario: find a bay, start parking, wait until parked.
Outcome parkFrom(double x, double y, double speed, double lotStartY) {
    Simulation sim;
    setup(sim, x, y, speed, lotStartY);
    Outcome o;
    if (!waitReady(sim)) return o;
    sim.adas().request(AdasRequest::StartParking);
    o.parked = runUntil(sim, 150.0, [&] { return phase(sim) == ParkingPhase::Parked; });
    const VehicleState& s = sim.vehicle().state();
    for (const SlotReading& r : sim.sensors().bays)
        if (r.id == sim.adas().status().parkingSlot) o.inside = ParkingAssist::insideSlot(s, sim.vehicle().params(), r, 0.1);
    o.headingErrorDeg = std::abs(s.heading + kHalfPi) * 180.0 / 3.14159265358979323846;
    o.collisions = sim.collisions();
    o.time = s.time;
    return o;
}
}  // namespace

class ParkingTest : public QObject {
    Q_OBJECT
private slots:
    void lotGeometry() {
        World w;
        const int first = w.addParkingRow(20.0, 8, kLot);
        QCOMPARE(first, 1);
        QCOMPARE(w.bays().size(), size_t(8));
        QCOMPARE(w.bays()[1].y - w.bays()[0].y, 3.0);
        QCOMPARE(w.bays()[0].x, 5.4 + 3.0 + 2.75);
        QCOMPARE(w.objects().size(), size_t(6));  // parked cars in the six occupied bays
    }

    void spawnedLotPutsFirstFreeBayAhead() {
        Simulation sim;
        sim.vehicle().reset(3.6, 10.0, 0.0, 8.0);
        sim.spawnParkingLot(30.0);
        QCOMPARE(sim.world().bays().size(), size_t(8));
        QCOMPARE(sim.world().bays()[3].y, 40.0);  // bay 4: 30 m ahead of the car at y = 10
        sim.spawnParkingLot(20.0);                 // replaces, never stacks
        QCOMPARE(sim.world().bays().size(), size_t(8));
        QCOMPARE(sim.world().objects().size(), size_t(6));
        QCOMPARE(sim.world().bays()[3].y, 30.0);
    }

    void lotIsUsableRightAwayWhileDriving() {
        Simulation sim;
        sim.vehicle().reset(3.6, 0.0, 0.0, 8.0);
        sim.driver().ignition = true;
        sim.driver().throttle = 0.3;  // still driving when the lot appears
        sim.spawnParkingLot(34.0);
        sim.adas().request(AdasRequest::ToggleParkingAssist);
        QVERIFY(runUntil(sim, 3.0, [&] { return phase(sim) == ParkingPhase::Ready; }));
    }

    void sensorsReportFreeAndOccupiedBays() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 10.0);
        sim.step(kDt);
        QCOMPARE(sim.sensors().bays.size(), size_t(8));
        int freeCount = 0;
        for (const SlotReading& r : sim.sensors().bays) {
            if (r.free) {
                ++freeCount;
                QVERIFY(r.id == 4 || r.id == 7);
            }
        }
        QCOMPARE(freeCount, 2);
    }

    void slotsOutOfRangeAreNotSeen() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 100.0);
        sim.step(kDt);
        QVERIFY(sim.sensors().bays.empty());
    }

    void noPlanWhenEverythingIsOccupied() {
        Simulation sim;
        sim.vehicle().reset(3.6, 0, 0, 0);
        sim.driver().ignition = true;
        sim.world().addParkingRow(20.0, 4, {true, true, true, true});
        sim.adas().request(AdasRequest::ToggleParkingAssist);
        run(sim, 1.0);
        QCOMPARE(int(phase(sim)), int(ParkingPhase::Searching));
        sim.adas().request(AdasRequest::StartParking);
        run(sim, 11.0);  // waits for a bay, then gives up
        QVERIFY(hasEvent(sim, "Parking cannot start"));
        QVERIFY(!sim.adas().status().parkingActive);
    }

    void pressingStartAloneSwitchesOnAndParks() {
        // Driver presses only L (lot) and Y (start): no separate "assist on" step.
        Simulation sim;
        sim.vehicle().reset(3.6, 0.0, 0.0, 0.0);
        sim.driver().ignition = true;
        sim.spawnParkingLot(25.0);
        sim.adas().request(AdasRequest::StartParking);
        QVERIFY(runUntil(sim, 5.0, [&] { return sim.adas().status().parkingActive; }));
        QVERIFY(runUntil(sim, 150.0, [&] { return phase(sim) == ParkingPhase::Parked; }));
        QCOMPARE(sim.collisions(), 0);
    }

    void startWaitsForTheBayToComeIntoRange() {
        Simulation sim;
        sim.vehicle().reset(3.6, 0.0, 0.0, 0.0);
        sim.driver().ignition = true;
        sim.world().addParkingRow(55.0, 8, kLot);  // free bays are beyond the 40 m sensor range at first
        sim.adas().request(AdasRequest::StartParking);
        run(sim, 2.0);
        QVERIFY(!sim.adas().status().parkingActive);
        sim.vehicle().reset(3.6, 30.0, 0.0, 0.0);  // moved up the road
        QVERIFY(runUntil(sim, 5.0, [&] { return sim.adas().status().parkingActive; }));
    }

    void planEndsInsideTheBayWithClearance() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0);
        QVERIFY(waitReady(sim));
        const ParkingPlan& plan = sim.adas().parking().plan();
        QVERIFY(plan.valid);
        QVERIFY(plan.slotId == 4 || plan.slotId == 7);
        QVERIFY(plan.radius >= sim.vehicle().params().wheelbase / std::tan(sim.vehicle().params().maxSteerAngle) - 1e-9);
        QVERIFY(plan.straight >= 0.0);
        QVERIFY(plan.clearance >= 0.2);
        const ParkingPose& last = plan.path.back();
        VehicleState end;
        end.x = last.x;
        end.y = last.y;
        end.heading = last.heading;
        SlotReading slot;
        for (const SlotReading& r : sim.sensors().bays)
            if (r.id == plan.slotId) slot = r;
        QVERIFY(ParkingAssist::insideSlot(end, sim.vehicle().params(), slot));
    }

    void parksFromRightLane() {
        const Outcome o = parkFrom(3.6, 0.0, 0.0, 25.0);
        QVERIFY(o.parked);
        QVERIFY(o.inside);
        QVERIFY2(o.headingErrorDeg < 6.0, qPrintable(QString("heading error %1 deg").arg(o.headingErrorDeg)));
        QCOMPARE(o.collisions, 0);
        QVERIFY2(o.time < 120.0, qPrintable(QString("took %1 s").arg(o.time)));
    }

    void parksFromOtherLateralPositions() {
        for (double x : {2.0, 3.0, 5.0}) {
            const Outcome o = parkFrom(x, 0.0, 0.0, 25.0);
            QVERIFY2(o.parked && o.inside && o.collisions == 0 && o.headingErrorDeg < 6.0,
                     qPrintable(QString("x=%1 parked=%2 inside=%3 coll=%4 hdg=%5")
                                    .arg(x).arg(o.parked).arg(o.inside).arg(o.collisions).arg(o.headingErrorDeg)));
        }
    }

    void parksWhileStillRolling() {
        const Outcome o = parkFrom(3.6, 0.0, 3.0, 25.0);
        QVERIFY(o.parked && o.inside);
        QCOMPARE(o.collisions, 0);
    }

    void parksWhenStartingPastTheBay() {
        // Car is beyond the bays and has to back up along the road to the arc start point.
        const Outcome o = parkFrom(3.6, 60.0, 0.0, 20.0);
        QVERIFY(o.parked && o.inside);
        QCOMPARE(o.collisions, 0);
    }

    void parksWhenStartedAtFortyKmh() {
        const Outcome o = parkFrom(3.6, 0.0, 10.0, 30.0);  // 36 km/h, lot already beside the car
        QVERIFY(o.parked && o.inside);
        QCOMPARE(o.collisions, 0);
    }

    void parksFromHighwaySpeed() {
        const Outcome o = parkFrom(3.6, 0.0, 22.0, 38.0);  // ~80 km/h: the assist brakes, and reverses back if it overshoots
        QVERIFY(o.parked && o.inside);
        QCOMPARE(o.collisions, 0);
    }

    void heldKeysAtStartDoNotCancelParking() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0);
        sim.driver().throttle = 1.0;  // driver is still holding W when pressing Y
        sim.driver().steer = 0.0;
        QVERIFY(waitReady(sim));
        sim.adas().request(AdasRequest::StartParking);
        run(sim, 6.0);
        QVERIFY(sim.adas().status().parkingActive);   // not cancelled by the held throttle
        QVERIFY(!hasEvent(sim, "driver takeover"));
        sim.driver().throttle = 0.0;                  // released ...
        run(sim, 0.5);
        sim.driver().brake = 1.0;                     // ... then braking is a real takeover
        run(sim, 0.3);
        QVERIFY(!sim.adas().status().parkingActive);
        QVERIFY(hasEvent(sim, "driver takeover"));
    }

    void pressingStartTwiceKeepsParking() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0);
        sim.adas().request(AdasRequest::StartParking);
        run(sim, 2.0);
        QVERIFY(sim.adas().status().parkingActive);
        sim.adas().request(AdasRequest::StartParking);
        QVERIFY(runUntil(sim, 150.0, [&] { return phase(sim) == ParkingPhase::Parked; }));
        QCOMPARE(sim.collisions(), 0);
    }

    void startRefusedWhenTooFast() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 33.0, 60.0);
        sim.driver().throttle = 0.1;
        QVERIFY(waitReady(sim));
        sim.adas().request(AdasRequest::StartParking);
        run(sim, 0.5);
        QVERIFY(hasEvent(sim, "slow down"));
        QVERIFY(!sim.adas().status().parkingActive);
    }

    void notReadyWhenHeadingIsOff() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0, 0.4);
        sim.adas().request(AdasRequest::ToggleParkingAssist);
        run(sim, 1.0);
        QCOMPARE(int(phase(sim)), int(ParkingPhase::Searching));
    }

    void assistRefusedOffTheRoad() {
        Simulation sim;
        setup(sim, 8.0, 0.0, 0.0, 25.0);  // beside the bay entrance, outside the lane markings
        sim.adas().request(AdasRequest::ToggleParkingAssist);
        run(sim, 1.0);
        QCOMPARE(int(phase(sim)), int(ParkingPhase::Off));
        QVERIFY(hasEvent(sim, "not on the road"));
    }

    void assistNeedsIgnition() {
        Simulation sim;
        sim.vehicle().reset(3.6, 0, 0, 0);
        sim.world().addParkingRow(25.0, 8, kLot);
        sim.adas().request(AdasRequest::ToggleParkingAssist);
        run(sim, 0.5);
        QVERIFY(!sim.adas().status().parkingEnabled);
        QVERIFY(hasEvent(sim, "Parking assist unavailable"));
    }

    void driverTakeoverAbortsParking() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0);
        QVERIFY(waitReady(sim));
        sim.adas().request(AdasRequest::StartParking);
        QVERIFY(runUntil(sim, 60.0, [&] { return phase(sim) == ParkingPhase::Arc; }));
        QVERIFY(sim.adas().status().parkingActive);
        sim.driver().brake = 1.0;
        run(sim, 0.2);
        QVERIFY(!sim.adas().status().parkingActive);
        QVERIFY(hasEvent(sim, "driver takeover"));
        sim.driver().brake = 1.0;
        run(sim, 3.0);
        QCOMPARE(sim.collisions(), 0);
    }

    void ignitionOffAbortsParking() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0);
        QVERIFY(waitReady(sim));
        sim.adas().request(AdasRequest::StartParking);
        run(sim, 2.0);
        sim.driver().ignition = false;
        run(sim, 0.2);
        QVERIFY(!sim.adas().status().parkingActive);
        QVERIFY(!sim.adas().status().parkingEnabled);
    }

    void obstacleOnPlannedPathAbortsWithoutContact() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0);
        QVERIFY(waitReady(sim));
        sim.adas().request(AdasRequest::StartParking);
        QVERIFY(runUntil(sim, 60.0, [&] { return phase(sim) == ParkingPhase::Arc; }));
        run(sim, 1.0);
        // Drop a box across the planned path a few metres ahead of the car.
        const ParkingPlan plan = sim.adas().parking().plan();
        const ParkingPose& p = plan.path[plan.path.size() * 2 / 3];
        WorldObject box = World::makeObstacle(p.x, p.y);
        box.width = 1.5;
        box.length = 1.5;
        sim.world().add(box);
        run(sim, 30.0);
        QVERIFY(hasEvent(sim, "Parking aborted"));
        QVERIFY(!sim.adas().status().parkingActive);
        QCOMPARE(sim.collisions(), 0);
    }

    void obstacleOnTheRoadWhileAligningStopsTheCar() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0);
        QVERIFY(waitReady(sim));
        sim.adas().request(AdasRequest::StartParking);
        run(sim, 1.5);
        sim.world().add(World::makeObstacle(3.6, sim.vehicle().state().y + 14.0));
        run(sim, 25.0);
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(sim.vehicle().state().speed < 0.1);
    }

    void startingParkingTurnsOffOtherAssists() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0);
        sim.adas().request(AdasRequest::ToggleLka);
        run(sim, 0.2);
        QVERIFY(sim.adas().status().lkaEnabled);
        QVERIFY(waitReady(sim));
        sim.adas().request(AdasRequest::StartParking);
        run(sim, 0.2);
        QVERIFY(!sim.adas().status().lkaEnabled);
        QVERIFY(sim.adas().status().parkingActive);
        flush(sim);
        sim.adas().request(AdasRequest::ToggleAcc);
        run(sim, 0.2);
        QVERIFY(!sim.adas().status().accEnabled);
        QVERIFY(hasEvent(sim, "Parking in progress"));
    }

    void staysPutOncePerked() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0);
        QVERIFY(waitReady(sim));
        sim.adas().request(AdasRequest::StartParking);
        QVERIFY(runUntil(sim, 150.0, [&] { return phase(sim) == ParkingPhase::Parked; }));
        const double x = sim.vehicle().state().x, y = sim.vehicle().state().y;
        run(sim, 5.0);
        QVERIFY(std::abs(sim.vehicle().state().x - x) < 0.01);
        QVERIFY(std::abs(sim.vehicle().state().y - y) < 0.01);
        QVERIFY(std::abs(sim.vehicle().state().speed) < 0.01);
    }

    void switchingAssistOffClearsTheState() {
        Simulation sim;
        setup(sim, 3.6, 0.0, 0.0, 25.0);
        QVERIFY(waitReady(sim));
        sim.adas().request(AdasRequest::ToggleParkingAssist);
        run(sim, 0.2);
        QCOMPARE(int(phase(sim)), int(ParkingPhase::Off));
        QVERIFY(!sim.adas().status().parkingEnabled);
    }

    void deterministic() {
        const Outcome a = parkFrom(3.6, 0.0, 0.0, 25.0);
        const Outcome b = parkFrom(3.6, 0.0, 0.0, 25.0);
        QCOMPARE(a.time, b.time);
        QCOMPARE(a.headingErrorDeg, b.headingErrorDeg);
    }
};

QTEST_APPLESS_MAIN(ParkingTest)
#include "tst_parking.moc"
