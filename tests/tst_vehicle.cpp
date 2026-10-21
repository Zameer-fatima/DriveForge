#include <QtTest>
#include <cmath>

#include "VehicleModel.h"

using namespace driveforge;

namespace {
constexpr double kDt = 1.0 / 120.0;

void run(VehicleModel& m, double seconds) {
    const int n = static_cast<int>(seconds / kDt);
    for (int i = 0; i < n; ++i) m.step(kDt);
}

ControlInput drive(double throttle, double brake = 0.0, double steer = 0.0, Gear g = Gear::Drive) {
    ControlInput in;
    in.ignition = true;
    in.throttle = throttle;
    in.brake = brake;
    in.steer = steer;
    in.gear = g;
    return in;
}
}  // namespace

class VehicleTest : public QObject {
    Q_OBJECT
private slots:
    void acceleratesFromRest() {
        VehicleModel m;
        m.setInput(drive(1.0));
        run(m, 5.0);
        QVERIFY(m.state().speed > 12.0);
        QVERIFY(m.state().speed < m.params().maxAccel * 5.0);  // can never beat a_max
        QVERIFY(m.state().y > 0.0);
    }

    void topSpeedIsBounded() {
        VehicleModel m;
        m.setInput(drive(1.0));
        run(m, 120.0);
        QVERIFY(m.state().speed <= m.params().maxSpeedForward);
        QVERIFY(m.state().speed > 0.7 * m.params().maxSpeedForward);  // drag sets the real top speed (~41 m/s)
    }

    void brakingDistanceMatchesFormula() {
        VehicleModel m;
        m.reset(0, 0, 0, 20.0);
        const double predicted = m.brakingDistance(20.0);  // 25 m
        QCOMPARE_NE(predicted, 0.0);
        m.setInput(drive(0.0, 1.0));
        run(m, 10.0);
        QVERIFY(std::abs(m.state().speed) < 1e-9);
        // Real stop includes brake ramp-up (+~2.5 m) and drag (-~1 m).
        QVERIFY2(m.state().y > predicted - 2.0 && m.state().y < predicted + 4.0,
                 qPrintable(QString("stopped after %1 m, predicted %2 m").arg(m.state().y).arg(predicted)));
    }

    void brakingNeverReverses() {
        VehicleModel m;
        m.reset(0, 0, 0, 5.0);
        m.setInput(drive(0.0, 1.0));
        run(m, 5.0);
        QVERIFY(m.state().speed >= 0.0);
    }

    void turningRadiusMatchesBicycleGeometry() {
        VehicleModel m;
        m.reset(0, 0, 0, 10.0);
        m.setInput(drive(0.1, 0.0, 0.2));  // gentle right turn, speed roughly constant
        run(m, 1.0);                       // let steering settle
        const double delta = m.state().steerAngle;
        QVERIFY(delta > 0.1);
        const double radius = m.params().wheelbase / std::tan(delta);
        const double x0 = m.state().x, y0 = m.state().y, h0 = m.state().heading;
        // Turn centre is perpendicular-right of heading.
        const double cx = x0 + radius * std::cos(h0);
        const double cy = y0 - radius * std::sin(h0);
        run(m, 3.0);
        const double dist = std::hypot(m.state().x - cx, m.state().y - cy);
        QVERIFY2(std::abs(dist - radius) < 0.03 * radius,
                 qPrintable(QString("distance %1 vs radius %2").arg(dist).arg(radius)));
        QVERIFY(m.state().heading > h0);  // clockwise
    }

    void steeringLimitedAtSpeed() {
        VehicleModel m;
        m.reset(0, 0, 0, 40.0);
        m.setInput(drive(0.0, 0.0, 1.0));
        run(m, 3.0);
        const double limit = m.steerLimit(m.state().speed);
        QVERIFY(limit < 0.05);
        QVERIFY(m.state().steerAngle <= limit + 1e-9);
        QVERIFY(std::abs(m.state().lateralAcceleration) <= m.params().maxLateralAccel + 0.1);
    }

    void gearChangeOnlyWhenStopped() {
        VehicleModel m;
        m.reset(0, 0, 0, 5.0);
        m.setInput(drive(0.0, 0.0, 0.0, Gear::Reverse));
        run(m, 0.2);
        QCOMPARE(m.gear(), Gear::Drive);
        m.setInput(drive(0.0, 1.0, 0.0, Gear::Reverse));
        run(m, 3.0);
        QCOMPARE(m.gear(), Gear::Reverse);
    }

    void reversesAndIsBounded() {
        VehicleModel m;
        m.setInput(drive(1.0, 0.0, 0.0, Gear::Reverse));
        run(m, 20.0);
        QCOMPARE(m.gear(), Gear::Reverse);
        QVERIFY(m.state().speed < -1.0);
        QVERIFY(m.state().speed >= -m.params().maxSpeedReverse);
        QVERIFY(m.state().y < 0.0);
    }

    void ignitionOffIgnoresThrottle() {
        VehicleModel m;
        ControlInput in = drive(1.0);
        in.ignition = false;
        m.setInput(in);
        run(m, 2.0);
        QCOMPARE(m.state().speed, 0.0);
    }

    void coastingSlowsToStop() {
        VehicleModel m;
        m.reset(0, 0, 0, 10.0);
        m.setInput(drive(0.0));
        run(m, 120.0);
        QCOMPARE(m.state().speed, 0.0);
    }

    void deterministic() {
        VehicleModel a, b;
        for (VehicleModel* m : {&a, &b}) {
            m->setInput(drive(0.7, 0.0, 0.3));
            run(*m, 4.0);
        }
        QCOMPARE(a.state().x, b.state().x);
        QCOMPARE(a.state().y, b.state().y);
        QCOMPARE(a.state().heading, b.state().heading);
    }

    void ignoresInvalidTimestep() {
        VehicleModel m;
        m.setInput(drive(1.0));
        m.step(0.0);
        m.step(-1.0);
        QCOMPARE(m.state().time, 0.0);
    }
};

QTEST_APPLESS_MAIN(VehicleTest)
#include "tst_vehicle.moc"
