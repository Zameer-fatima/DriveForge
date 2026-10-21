#include <QtTest>
#include <cmath>

#include "Simulation.h"

using namespace driveforge;

namespace {
constexpr double kDt = 1.0 / 120.0;

void run(Simulation& sim, double seconds) {
    const int n = static_cast<int>(seconds / kDt);
    for (int i = 0; i < n; ++i) sim.step(kDt);
}

// Ignition on, already moving at `speedKmh`, driver input held.
void startMoving(Simulation& sim, double speedKmh, double throttle = 0.0, double x = 0.0) {
    sim.vehicle().reset(x, 0.0, 0.0, speedKmh / 3.6);
    sim.driver().ignition = true;
    sim.driver().throttle = throttle;
}

bool hasEvent(Simulation& sim, const char* text, std::vector<Event>* sink = nullptr) {
    static std::vector<Event> all;
    for (Event& e : sim.takeEvents()) all.push_back(e);
    if (sink) *sink = all;
    for (const Event& e : all)
        if (e.message.find(text) != std::string::npos) return true;
    return false;
}
void clearEvents(Simulation& sim) { sim.takeEvents(); hasEvent(sim, "\x01"); }

// ---- helpers for the autonomous overtaking tests ----
struct Drive {
    double maxLat = 0.0, minSpeed = 1e9, minX = 1e9, maxX = -1e9;
    bool aeb = false;
    std::vector<std::string> events;
};
bool saw(const Drive& d, const char* text) {
    for (const std::string& e : d.events)
        if (e.find(text) != std::string::npos) return true;
    return false;
}
// Steps the simulation, tracking comfort and position, collecting event texts. `each` runs before every step.
template <typename F>
Drive driveFor(Simulation& sim, double seconds, F each) {
    Drive d;
    const int n = static_cast<int>(seconds / kDt);
    for (int i = 0; i < n; ++i) {
        each(sim);
        sim.step(kDt);
        const VehicleState& s = sim.vehicle().state();
        d.maxLat = std::max(d.maxLat, std::abs(s.lateralAcceleration));
        d.minSpeed = std::min(d.minSpeed, s.speed);
        d.minX = std::min(d.minX, s.x);
        d.maxX = std::max(d.maxX, s.x);
        d.aeb = d.aeb || sim.adas().status().aebBraking;
        for (const Event& e : sim.takeEvents()) d.events.push_back(e.message);
    }
    return d;
}
void startAutonomous(Simulation& sim, double kmh) {
    sim.driver().ignition = true;
    sim.vehicle().reset(0.0, 0.0, 0.0, kmh / 3.6);
    sim.adas().request(AdasRequest::ToggleAutonomous);
}

}  // namespace

class AdasTest : public QObject {
    Q_OBJECT
private slots:
    // ---- road / world / sensors ----
    void roadLanes() {
        Road r;
        QCOMPARE(r.laneCenter(0), -3.6);
        QCOMPARE(r.laneCenter(1), 0.0);
        QCOMPARE(r.laneCenter(2), 3.6);
        QCOMPARE(r.laneIndexAt(0.4), 1);
        QCOMPARE(r.laneIndexAt(2.2), 2);
        QCOMPARE(r.laneIndexAt(-50.0), 0);
        QVERIFY(r.laneVisible(5.0));
        QVERIFY(!r.laneVisible(9.0));
    }

    void overlapDetection() {
        VehicleModel m;
        m.reset(0, 0, 0, 0);
        WorldObject ahead = World::makeObstacle(0, 3.0);  // ego half length 2.25, obstacle half length 0.5
        QVERIFY(!overlaps(m.state(), m.params(), ahead));
        ahead.y = 2.6;
        QVERIFY(overlaps(m.state(), m.params(), ahead));
        WorldObject side = World::makeObstacle(1.9, 0);
        QVERIFY(!overlaps(m.state(), m.params(), side));
        side.x = 1.5;
        QVERIFY(overlaps(m.state(), m.params(), side));
    }

    void pedestrianWaitsForTrigger() {
        World w;
        VehicleState ego;
        ego.speed = 10.0;
        const int id = w.add(World::makePedestrian(6.0, 100.0, -1.4, 3.0));
        w.step(0.1, ego);
        QCOMPARE(w.objects().front().x, 6.0);  // 10 s away: still waiting
        ego.y = 75.0;                          // 2.5 s away
        w.step(0.1, ego);
        QVERIFY(w.objects().front().x < 6.0);
        QCOMPARE(w.objects().front().id, id);
    }

    void sensorReportsGeometry() {
        World w;
        VehicleModel m;
        m.reset(0, 0, 0, 20.0);
        w.add(World::makeVehicle(0.5, 50.0, 10.0));
        const SensorReadings r = sense(w, m.state(), m.params());
        QCOMPARE(r.detections.size(), size_t(1));
        const Detection& d = r.detections.front();
        QVERIFY(std::abs(d.gap - (50.0 - 4.5)) < 1e-9);
        QVERIFY(std::abs(d.lateral - 0.5) < 1e-9);
        QVERIFY(std::abs(d.closingSpeed - 10.0) < 1e-9);
        QVERIFY(r.cameraOk);
        QCOMPARE(r.lane.laneIndex, 1);
    }

    void sensorRangeLimits() {
        World w;
        VehicleModel m;
        w.add(World::makeObstacle(0, 200.0));
        w.add(World::makeObstacle(0, -40.0));
        QVERIFY(sense(w, m.state(), m.params()).detections.empty());
    }

    // ---- AEB ----
    void aebStopsForStaticObstacleAtAllSpeeds() {
        for (double kmh : {30.0, 50.0, 80.0, 100.0, 120.0}) {
            Simulation sim;
            startMoving(sim, kmh, 1.0);  // driver keeps flooring the throttle
            sim.spawnObstacle(160.0);
            run(sim, 30.0);
            QVERIFY2(sim.collisions() == 0, qPrintable(QString("collision at %1 km/h").arg(kmh)));
            QVERIFY(sim.vehicle().state().speed < 0.1);
            QVERIFY(sim.frontGap() > 0.2);
        }
    }

    void aebStopsForSlowLeadVehicle() {
        Simulation sim;
        startMoving(sim, 90.0, 1.0);
        sim.spawnLeadVehicle(70.0, 5.0);
        run(sim, 20.0);
        QCOMPARE(sim.collisions(), 0);
    }

    void aebWithoutItCollides() {
        Simulation sim;
        startMoving(sim, 50.0);
        sim.adas().request(AdasRequest::ToggleAeb);
        sim.spawnObstacle(100.0);
        run(sim, 15.0);
        QCOMPARE(sim.collisions(), 1);
        QVERIFY(hasEvent(sim, "COLLISION with obstacle"));
        QCOMPARE(sim.vehicle().state().speed, 0.0);
    }

    void aebAvoidsCrossingPedestrian() {
        for (double kmh : {30.0, 50.0, 70.0}) {
            Simulation sim;
            startMoving(sim, kmh);
            sim.spawnPedestrian(kmh / 3.6 * 7.0);  // ~7 s away, so the car is still at speed when the pedestrian steps out
            run(sim, 20.0);
            QVERIFY2(sim.collisions() == 0, qPrintable(QString("pedestrian hit at %1 km/h").arg(kmh)));
        }
    }

    void aebAvoidsPedestrianSteppingIntoTheFlank() {
        // Regression: the pedestrian reaches the path only after our front bumper has passed, while our flank is still alongside.
        for (double ttc : {4.0, 4.5}) {
            Simulation sim;
            startMoving(sim, 50.0, 0.1);
            sim.spawnPedestrian(55.0, ttc);
            run(sim, 25.0);
            QVERIFY2(sim.collisions() == 0, qPrintable(QString("hit at trigger TTC %1").arg(ttc)));
        }
    }

    void aebIgnoresPedestrianWalkingAway() {
        Simulation sim;
        startMoving(sim, 50.0, 0.3);
        sim.world().add(World::makePedestrian(8.0, 80.0, +1.4, -1.0));  // moves away from the road
        double minSpeed = 1e9;
        for (int i = 0; i < 120 * 8; ++i) {
            sim.step(kDt);
            minSpeed = std::min(minSpeed, sim.vehicle().state().speed);
            QVERIFY(!sim.adas().status().aebBraking);
        }
        QVERIFY(minSpeed > 10.0);
    }

    void aebIgnoresAdjacentLaneObstacle() {
        Simulation sim;
        startMoving(sim, 90.0, 0.2);
        sim.world().add(World::makeObstacle(3.6, 100.0));
        for (int i = 0; i < 120 * 6; ++i) {
            sim.step(kDt);
            QVERIFY(!sim.adas().status().aebWarning);
        }
        QCOMPARE(sim.collisions(), 0);
    }

    void aebDoesNotBrakeInReverse() {
        Simulation sim;
        sim.driver().ignition = true;
        sim.driver().gear = Gear::Reverse;
        sim.driver().throttle = 1.0;
        sim.world().add(World::makeObstacle(0, -20.0));
        run(sim, 5.0);
        QVERIFY(!sim.adas().status().aebBraking);
    }

    // ---- ACC ----
    void accHoldsSetSpeed() {
        Simulation sim;
        startMoving(sim, 72.0);
        sim.adas().request(AdasRequest::ToggleAcc);
        run(sim, 30.0);
        QVERIFY(sim.adas().status().accEnabled);
        QVERIFY(std::abs(sim.vehicle().state().speed - sim.adas().status().accSetSpeed) < 0.5);
    }

    void accAcceleratesToHigherSetSpeed() {
        Simulation sim;
        startMoving(sim, 50.0);
        sim.adas().request(AdasRequest::ToggleAcc);
        run(sim, 0.1);
        for (int i = 0; i < 4; ++i) sim.adas().request(AdasRequest::AccSpeedUp);
        run(sim, 40.0);
        QVERIFY(std::abs(sim.vehicle().state().speed - sim.adas().status().accSetSpeed) < 0.7);
    }

    void accFollowsSlowerLead() {
        Simulation sim;
        startMoving(sim, 108.0);
        sim.adas().request(AdasRequest::ToggleAcc);
        sim.spawnLeadVehicle(100.0, 20.0);
        double minGap = 1e9;
        for (int i = 0; i < 120 * 90; ++i) {
            sim.step(kDt);
            minGap = std::min(minGap, sim.frontGap());
            QVERIFY(!sim.adas().status().aebBraking);  // comfortable: AEB never needed
        }
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(std::abs(sim.vehicle().state().speed - 20.0) < 0.5);
        const double desired = 5.0 + sim.adas().status().accTimeGap * 20.0;
        QVERIFY2(std::abs(sim.frontGap() - desired) < 6.0,
                 qPrintable(QString("gap %1 desired %2").arg(sim.frontGap()).arg(desired)));
        QVERIFY(minGap > 20.0);
    }

    void accStopsBehindStationaryObstacle() {
        Simulation sim;
        startMoving(sim, 54.0);
        sim.adas().request(AdasRequest::ToggleAcc);
        sim.spawnObstacle(150.0);
        run(sim, 60.0);
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(sim.vehicle().state().speed < 0.1);
        QVERIFY2(std::abs(sim.frontGap() - 5.0) < 2.0, qPrintable(QString("gap %1").arg(sim.frontGap())));
    }

    void accCancelledByDriverBrake() {
        Simulation sim;
        startMoving(sim, 72.0);
        sim.adas().request(AdasRequest::ToggleAcc);
        run(sim, 1.0);
        QVERIFY(sim.adas().status().accEnabled);
        sim.driver().brake = 1.0;
        run(sim, 0.1);
        QVERIFY(!sim.adas().status().accEnabled);
    }

    void accUnavailableWhenIgnitionOff() {
        Simulation sim;
        sim.vehicle().reset(0, 0, 0, 20.0);
        sim.adas().request(AdasRequest::ToggleAcc);
        run(sim, 0.1);
        QVERIFY(!sim.adas().status().accEnabled);
        QVERIFY(hasEvent(sim, "ACC unavailable"));
    }

    // ---- Lane keeping / lane change ----
    void lkaCentresLane() {
        Simulation sim;
        startMoving(sim, 90.0, 0.2, 1.2);
        sim.vehicle().reset(1.2, 0, 0.1, 25.0);
        sim.driver().ignition = true;
        sim.driver().throttle = 0.2;
        sim.adas().request(AdasRequest::ToggleLka);
        double maxLat = 0.0;
        for (int i = 0; i < 120 * 20; ++i) {
            sim.step(kDt);
            maxLat = std::max(maxLat, std::abs(sim.vehicle().state().lateralAcceleration));
        }
        QVERIFY(sim.adas().status().lkaEnabled);
        QVERIFY2(std::abs(sim.vehicle().state().x) < 0.1, qPrintable(QString("x=%1").arg(sim.vehicle().state().x)));
        QVERIFY(std::abs(sim.vehicle().state().heading) < 0.02);
        QVERIFY2(maxLat < 3.0, qPrintable(QString("lateral accel %1").arg(maxLat)));
    }

    void lkaWorksAtLowAndHighSpeed() {
        for (double kmh : {20.0, 60.0, 130.0}) {
            Simulation sim;
            startMoving(sim, kmh, 0.0, -0.8);
            sim.vehicle().reset(-0.8, 0, 0.0, kmh / 3.6);
            sim.driver().ignition = true;
            sim.driver().throttle = kmh > 100 ? 0.3 : 0.0;
            sim.adas().request(AdasRequest::ToggleLka);
            run(sim, 12.0);
            QVERIFY2(std::abs(sim.vehicle().state().x) < 0.3, qPrintable(QString("%1 km/h x=%2").arg(kmh).arg(sim.vehicle().state().x)));
        }
    }

    void laneChangeToRight() {
        Simulation sim;
        startMoving(sim, 90.0, 0.2);
        sim.adas().request(AdasRequest::ToggleLka);
        run(sim, 1.0);
        sim.adas().request(AdasRequest::LaneChangeRight);
        double maxLat = 0.0;
        for (int i = 0; i < 120 * 10; ++i) {
            sim.step(kDt);
            maxLat = std::max(maxLat, std::abs(sim.vehicle().state().lateralAcceleration));
        }
        QVERIFY2(std::abs(sim.vehicle().state().x - 3.6) < 0.15, qPrintable(QString("x=%1").arg(sim.vehicle().state().x)));
        QVERIFY(!sim.adas().status().laneChanging);
        QVERIFY(hasEvent(sim, "Lane change complete"));
        QVERIFY2(maxLat < 3.0, qPrintable(QString("lateral accel %1").arg(maxLat)));
    }

    void laneChangeLeftThenRightBack() {
        Simulation sim;
        startMoving(sim, 90.0, 0.2);
        sim.adas().request(AdasRequest::ToggleLka);
        run(sim, 0.5);
        sim.adas().request(AdasRequest::LaneChangeLeft);
        run(sim, 8.0);
        QVERIFY(std::abs(sim.vehicle().state().x + 3.6) < 0.15);
        sim.adas().request(AdasRequest::LaneChangeRight);
        run(sim, 8.0);
        QVERIFY(std::abs(sim.vehicle().state().x) < 0.15);
    }

    void laneChangeRefusedAtRoadEdge() {
        Simulation sim;
        startMoving(sim, 90.0, 0.2, 3.6);
        sim.vehicle().reset(3.6, 0, 0, 25.0);
        sim.driver().ignition = true;
        sim.adas().request(AdasRequest::ToggleLka);
        run(sim, 0.5);
        sim.adas().request(AdasRequest::LaneChangeRight);
        run(sim, 0.5);
        QVERIFY(hasEvent(sim, "no lane in that direction"));
        QVERIFY(std::abs(sim.vehicle().state().x - 3.6) < 0.2);
    }

    void laneChangeRefusedWhenTargetLaneOccupied() {
        Simulation sim;
        startMoving(sim, 90.0, 0.2);
        sim.world().add(World::makeVehicle(3.6, 5.0, 25.0));  // alongside in the right lane
        sim.adas().request(AdasRequest::ToggleLka);
        run(sim, 0.5);
        sim.adas().request(AdasRequest::LaneChangeRight);
        run(sim, 0.5);
        QVERIFY(hasEvent(sim, "occupied"));
        QVERIFY(std::abs(sim.vehicle().state().x) < 0.2);
    }

    void laneChangeRequiresLka() {
        Simulation sim;
        startMoving(sim, 90.0, 0.2);
        sim.adas().request(AdasRequest::LaneChangeRight);
        run(sim, 0.5);
        QVERIFY(hasEvent(sim, "needs Lane keeping"));
        QVERIFY(std::abs(sim.vehicle().state().x) < 0.2);
    }

    void lkaYieldsToDriverSteeringAndResumes() {
        Simulation sim;
        startMoving(sim, 90.0, 0.2);
        sim.adas().request(AdasRequest::ToggleLka);
        run(sim, 0.5);
        sim.driver().steer = 1.0;
        run(sim, 0.3);
        QVERIFY(sim.adas().status().lkaOverridden);
        sim.driver().steer = 0.0;
        run(sim, 15.0);
        QVERIFY(!sim.adas().status().lkaOverridden);
        QVERIFY(sim.adas().status().lkaEnabled);
        const double x = sim.vehicle().state().x;
        const Road r;
        QVERIFY(std::abs(x - r.laneCenter(r.laneIndexAt(x))) < 0.15);
    }

    void lkaUnavailableOffRoad() {
        Simulation sim;
        startMoving(sim, 50.0, 0.0, 12.0);
        sim.adas().request(AdasRequest::ToggleLka);
        run(sim, 0.2);
        QVERIFY(!sim.adas().status().lkaEnabled);
        QVERIFY(hasEvent(sim, "not visible"));
    }

    // ---- Autonomous mode ----
    void autonomousDrivesFromStandstillInLane() {
        Simulation sim;
        sim.driver().ignition = true;
        sim.adas().request(AdasRequest::ToggleAutonomous);
        run(sim, 45.0);
        QCOMPARE(int(sim.adas().status().mode), int(DriveMode::Autonomous));
        QVERIFY(std::abs(sim.vehicle().state().speed * 3.6 - 80.0) < 1.0);
        QVERIFY(std::abs(sim.vehicle().state().x) < 0.1);
    }

    void autonomousFollowsLeadAndAvoidsObstacle() {
        // This test is about following and stopping, so autonomous overtaking is switched off here.
        Simulation sim;
        sim.driver().ignition = true;
        sim.adas().request(AdasRequest::ToggleAutonomous);
        sim.adas().request(AdasRequest::ToggleOvertake);
        sim.spawnLeadVehicle(80.0, 10.0);
        run(sim, 90.0);
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(std::abs(sim.vehicle().state().speed - 10.0) < 0.5);

        Simulation sim2;
        sim2.driver().ignition = true;
        sim2.adas().request(AdasRequest::ToggleAutonomous);
        sim2.adas().request(AdasRequest::ToggleOvertake);
        sim2.spawnObstacle(600.0);
        run(sim2, 80.0);
        QCOMPARE(sim2.collisions(), 0);
        QVERIFY(sim2.vehicle().state().speed < 0.1);
    }

    void autonomousStopsForPedestrian() {
        Simulation sim;
        sim.driver().ignition = true;
        sim.adas().request(AdasRequest::ToggleAutonomous);
        run(sim, 20.0);
        sim.spawnPedestrian(100.0);
        run(sim, 25.0);
        QCOMPARE(sim.collisions(), 0);
    }

    void autonomousDriverTakeover() {
        Simulation sim;
        sim.driver().ignition = true;
        sim.adas().request(AdasRequest::ToggleAutonomous);
        run(sim, 5.0);
        sim.driver().brake = 1.0;
        run(sim, 0.1);
        QCOMPARE(int(sim.adas().status().mode), int(DriveMode::Manual));
        QVERIFY(!sim.adas().status().accEnabled);
        QVERIFY(!sim.adas().status().lkaEnabled);
        QVERIFY(hasEvent(sim, "takeover"));
    }

    void autonomousRefusedIgnitionOff() {
        Simulation sim;
        sim.adas().request(AdasRequest::ToggleAutonomous);
        run(sim, 0.1);
        QCOMPARE(int(sim.adas().status().mode), int(DriveMode::Manual));
    }

    void ignitionOffDisengagesAdas() {
        Simulation sim;
        startMoving(sim, 72.0);
        sim.adas().request(AdasRequest::ToggleAcc);
        run(sim, 0.5);
        sim.driver().ignition = false;
        run(sim, 0.1);
        QVERIFY(!sim.adas().status().accEnabled);
    }

    void resetClearsEverything() {
        Simulation sim;
        startMoving(sim, 50.0);
        sim.spawnObstacle(30.0);
        sim.adas().request(AdasRequest::ToggleAeb);
        run(sim, 5.0);
        sim.reset();
        QCOMPARE(sim.world().objects().size(), size_t(0));
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(sim.adas().status().aebArmed);
        QCOMPARE(sim.vehicle().state().speed, 0.0);
    }

    // ---- Autonomous overtaking and obstacle avoidance ----
    void autonomousOvertakesASlowerCarAndReturns() {
        Simulation sim;
        startAutonomous(sim, 80.0);
        const int leadId = sim.spawnLeadVehicle(80.0, 50.0 / 3.6);
        bool passedLead = false;
        const Drive d = driveFor(sim, 60.0, [&](Simulation& s) {
            const WorldObject* lead = s.world().find(leadId);
            if (lead && s.vehicle().state().y > lead->y + 6.0 && std::abs(s.vehicle().state().x) > 3.0) passedLead = true;
        });
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(d.minX < -3.0);                  // it used the left lane
        QVERIFY(passedLead);                     // and was fully past the car while in that lane
        QVERIFY(std::abs(sim.vehicle().state().x) < 0.2);  // and is back in its own lane
        QCOMPARE(int(sim.adas().status().overtakePhase), int(OvertakePhase::Idle));
        QVERIFY(!d.aeb);
        QVERIFY2(d.maxLat < 3.0, qPrintable(QString("lateral accel %1").arg(d.maxLat)));
        QVERIFY(saw(d, "overtaking a slower vehicle on the left"));
        QVERIFY(saw(d, "returning to the original lane"));
        QVERIFY(saw(d, "manoeuvre complete"));
    }

    void overtakingCanBeSwitchedOff() {
        Simulation sim;
        startAutonomous(sim, 80.0);
        sim.adas().request(AdasRequest::ToggleOvertake);
        sim.spawnLeadVehicle(80.0, 50.0 / 3.6);
        const Drive d = driveFor(sim, 45.0, [](Simulation&) {});
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(std::abs(d.minX) < 0.2 && std::abs(d.maxX) < 0.2);  // never left its lane
        QVERIFY(std::abs(sim.vehicle().state().speed - 50.0 / 3.6) < 0.7);  // follows at the lead's speed
        QVERIFY(saw(d, "overtaking OFF"));
    }

    void doesNotOvertakeACarThatIsBarelySlower() {
        Simulation sim;
        startAutonomous(sim, 80.0);
        sim.spawnLeadVehicle(80.0, 77.0 / 3.6);  // 3 km/h slower: not worth passing
        const Drive d = driveFor(sim, 40.0, [](Simulation&) {});
        QVERIFY(std::abs(d.minX) < 0.2 && std::abs(d.maxX) < 0.2);
    }

    void overtakesOnTheRightWhenTheLeftIsTaken() {
        Simulation sim;
        startAutonomous(sim, 80.0);
        sim.spawnLeadVehicle(80.0, 50.0 / 3.6);
        sim.world().add(World::makeVehicle(-3.6, 0.0, 80.0 / 3.6));  // keeps pace beside us on the left
        const Drive d = driveFor(sim, 60.0, [](Simulation&) {});
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(d.maxX > 3.0);       // used the right lane
        QVERIFY(d.minX > -0.5);      // never the left
        QVERIFY(std::abs(sim.vehicle().state().x) < 0.2);
        QVERIFY(saw(d, "on the right"));
    }

    void keepsFollowingWhenNeitherSideLaneIsFree() {
        Simulation sim;
        startAutonomous(sim, 80.0);
        const double slow = 50.0 / 3.6;
        sim.spawnLeadVehicle(80.0, slow);
        sim.world().add(World::makeVehicle(-3.6, 80.0, slow));  // slow cars abreast of the lead, in both side lanes
        sim.world().add(World::makeVehicle(3.6, 80.0, slow));
        const Drive d = driveFor(sim, 40.0, [](Simulation&) {});
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(std::abs(d.minX) < 0.2 && std::abs(d.maxX) < 0.2);
        QVERIFY(!saw(d, "Lane change refused"));  // no spam: the planner checks before asking
    }

    void obstacleAppearingAheadIsAvoidedByChangingLane() {
        Simulation sim;
        startAutonomous(sim, 80.0);
        bool spawned = false;
        const Drive d = driveFor(sim, 40.0, [&](Simulation& s) {
            if (!spawned && s.vehicle().state().time >= 3.0) {
                s.spawnObstacle(110.0);
                spawned = true;
            }
        });
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(d.minX < -3.0);                                // went round it
        QVERIFY2(d.minSpeed > 70.0 / 3.6, qPrintable(QString("slowest %1 km/h").arg(d.minSpeed * 3.6)));  // without braking hard
        QVERIFY(!d.aeb);
        QVERIFY(d.maxLat < 3.0);
        QVERIFY(std::abs(sim.vehicle().state().x) < 0.2);      // and returned
        QVERIFY(saw(d, "to avoid the obstacle"));
    }

    void obstacleWithNoFreeLaneMakesTheCarStop() {
        Simulation sim;
        startAutonomous(sim, 80.0);
        sim.world().add(World::makeObstacle(-3.6, 150.0));  // a wall of obstacles across all three lanes
        sim.world().add(World::makeObstacle(0.0, 150.0));
        sim.world().add(World::makeObstacle(3.6, 150.0));
        const Drive d = driveFor(sim, 60.0, [](Simulation&) {});
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(sim.vehicle().state().speed < 0.1);
        QVERIFY(std::abs(d.minX) < 0.2 && std::abs(d.maxX) < 0.2);  // no lane change attempted
        QVERIFY(saw(d, "no free lane"));
    }

    void obstacleTooCloseForALaneChangeIsHandledByBraking() {
        Simulation sim;
        startAutonomous(sim, 80.0);
        bool spawned = false;
        const Drive d = driveFor(sim, 30.0, [&](Simulation& s) {
            if (!spawned && s.vehicle().state().time >= 3.0) {
                s.spawnObstacle(50.0);  // too close to swerve round at this speed
                spawned = true;
            }
        });
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(std::abs(d.minX) < 0.2 && std::abs(d.maxX) < 0.2);  // it did not try
        QVERIFY(sim.vehicle().state().speed < 0.1);
        QVERIFY(d.aeb);
    }

    void obstacleDuringAnOvertakeIsStillAvoidedAtAnyMoment() {
        // The reported bug: with an obstacle placed while the car was mid-overtake (in the passing lane), it used to stop dead.
        for (double delay : {0.5, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 15.0}) {
            Simulation sim;
            sim.driver().ignition = true;
            sim.vehicle().reset(0.0, 0.0, 0.0, 0.0);
            sim.adas().request(AdasRequest::ToggleAutonomous);
            run(sim, 10.0);  // accelerate to the set speed
            sim.spawnLeadVehicle(60.0, 50.0 / 3.6);
            const double t0 = sim.vehicle().state().time;
            bool placed = false;
            const Drive d = driveFor(sim, 80.0, [&](Simulation& sm) {
                if (!placed && sm.vehicle().state().time - t0 >= delay) {
                    sm.spawnObstacle(120.0);
                    placed = true;
                }
            });
            const QString what = QString("obstacle %1 s after the lead car").arg(delay);
            QVERIFY2(sim.collisions() == 0, qPrintable(what + ": collision"));
            QVERIFY2(sim.vehicle().state().speed > 70.0 / 3.6, qPrintable(what + QString(": ended at %1 km/h").arg(sim.vehicle().state().speed * 3.6)));
            QVERIFY2(d.minSpeed > 2.0, qPrintable(what + QString(": stopped, slowest %1 km/h").arg(d.minSpeed * 3.6)));
            QVERIFY2(std::abs(sim.vehicle().state().x) < 0.3, qPrintable(what + QString(": not back in its lane, x = %1").arg(sim.vehicle().state().x)));
        }
    }

    void startingFromStandstillWithAnObstacleAheadStillGetsAround() {
        Simulation sim;
        startAutonomous(sim, 0.0);
        sim.spawnObstacle(80.0);
        const Drive d = driveFor(sim, 60.0, [](Simulation&) {});
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(sim.vehicle().state().y > 80.0 + 10.0);  // got past it
        QVERIFY(d.maxLat < 3.0);
    }

    void aLaneChangeTheDriverAsksForBecomesTheHomeLane() {
        Simulation sim;
        startAutonomous(sim, 80.0);
        run(sim, 2.0);
        sim.adas().request(AdasRequest::LaneChangeRight);
        run(sim, 10.0);
        QVERIFY(std::abs(sim.vehicle().state().x - 3.6) < 0.2);
        run(sim, 25.0);  // the planner must not drag the car back to the lane it started in
        QVERIFY(std::abs(sim.vehicle().state().x - 3.6) < 0.2);
    }

    void manualLaneKeepingAndAccNeverOvertake() {
        Simulation sim;
        sim.driver().ignition = true;
        sim.vehicle().reset(0.0, 0.0, 0.0, 80.0 / 3.6);
        sim.adas().request(AdasRequest::ToggleLka);
        sim.adas().request(AdasRequest::ToggleAcc);
        sim.spawnLeadVehicle(80.0, 50.0 / 3.6);
        const Drive d = driveFor(sim, 40.0, [](Simulation&) {});
        QVERIFY(std::abs(d.minX) < 0.2 && std::abs(d.maxX) < 0.2);
    }

    void driverSteeringCancelsAnAutonomousOvertake() {
        Simulation sim;
        startAutonomous(sim, 80.0);
        sim.spawnLeadVehicle(80.0, 50.0 / 3.6);
        int guard = 0;
        while (sim.adas().status().overtakePhase != OvertakePhase::ChangingOut && guard++ < 120 * 30) sim.step(kDt);
        QCOMPARE(int(sim.adas().status().overtakePhase), int(OvertakePhase::ChangingOut));
        sim.driver().steer = 0.5;  // the driver grabs the wheel
        run(sim, 0.2);
        QCOMPARE(int(sim.adas().status().mode), int(DriveMode::Manual));
        QCOMPARE(int(sim.adas().status().overtakePhase), int(OvertakePhase::Idle));
    }

    void overtakesFromTheLeftmostLaneUsingTheRight() {
        Simulation sim;
        sim.driver().ignition = true;
        sim.vehicle().reset(-3.6, 0.0, 0.0, 80.0 / 3.6);
        sim.adas().request(AdasRequest::ToggleAutonomous);
        sim.spawnLeadVehicle(80.0, 50.0 / 3.6);
        const Drive d = driveFor(sim, 60.0, [](Simulation&) {});
        QCOMPARE(sim.collisions(), 0);
        QVERIFY(d.maxX > -0.2);        // moved right, into the middle lane
        QVERIFY(d.minX > -3.8);        // never left the road on the left
        QVERIFY(std::abs(sim.vehicle().state().x + 3.6) < 0.2);  // back in the leftmost lane
    }

    void overtakingIsDeterministic() {
        double x[2], y[2];
        for (int k = 0; k < 2; ++k) {
            Simulation sim;
            startAutonomous(sim, 80.0);
            sim.spawnLeadVehicle(80.0, 50.0 / 3.6);
            run(sim, 40.0);
            x[k] = sim.vehicle().state().x;
            y[k] = sim.vehicle().state().y;
        }
        QCOMPARE(x[0], x[1]);
        QCOMPARE(y[0], y[1]);
    }

    void deterministicRuns() {
        double xs[2], ys[2];
        for (int k = 0; k < 2; ++k) {
            Simulation sim;
            sim.driver().ignition = true;
            sim.adas().request(AdasRequest::ToggleAutonomous);
            sim.spawnLeadVehicle(80.0, 12.0);
            sim.spawnPedestrian(300.0);
            run(sim, 30.0);
            xs[k] = sim.vehicle().state().x;
            ys[k] = sim.vehicle().state().y;
        }
        QCOMPARE(xs[0], xs[1]);
        QCOMPARE(ys[0], ys[1]);
    }
};

QTEST_APPLESS_MAIN(AdasTest)
#include "tst_adas.moc"
