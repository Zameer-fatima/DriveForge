#include <QtTest>

#include "EventDetector.h"
#include "RingBuffer.h"
#include "Telemetry.h"

using namespace driveforge;

namespace {
bool has(const std::vector<Event>& v, const char* text) {
    for (const Event& e : v)
        if (e.message.find(text) != std::string::npos) return true;
    return false;
}
VehicleState at(double t, double speed, double accel = 0.0, double lat = 0.0) {
    VehicleState s;
    s.time = t;
    s.speed = speed;
    s.acceleration = accel;
    s.lateralAcceleration = lat;
    return s;
}
}  // namespace

class TelemetryTest : public QObject {
    Q_OBJECT
private slots:
    void ringBufferOverwritesOldest() {
        RingBuffer<int> rb(3);
        QVERIFY(rb.empty());
        for (int i = 1; i <= 5; ++i) rb.push(i);
        QCOMPARE(rb.size(), size_t(3));
        QCOMPARE(rb.at(0), 3);
        QCOMPARE(rb.at(2), 5);
        rb.clear();
        QVERIFY(rb.empty());
        rb.push(9);
        QCOMPARE(rb.at(0), 9);
    }

    void samplesAtFixedPeriod() {
        Telemetry t(0.1, 200);
        int recorded = 0;
        for (int i = 0; i <= 1200; ++i) recorded += t.update(at(i / 120.0, 10.0)) ? 1 : 0;  // 10 s at 120 Hz
        QVERIFY(recorded >= 99 && recorded <= 101);
        QCOMPARE(t.samples().size(), size_t(recorded));
    }

    void capacityLimitsHistory() {
        Telemetry t(0.1, 20);
        for (int i = 0; i <= 1200; ++i) t.update(at(i / 120.0, 0.0));
        QCOMPARE(t.samples().size(), size_t(20));
        QVERIFY(t.samples().at(0).time < t.samples().at(19).time);
        QVERIFY(t.samples().at(19).time > 9.0);
    }

    void convertsUnitsAndClears() {
        Telemetry t;
        VehicleState s = at(0.0, 10.0, 1.5, 2.0);
        s.steerAngle = 3.14159265358979323846 / 6;
        QVERIFY(t.update(s));
        const TelemetrySample& x = t.samples().at(0);
        QCOMPARE_EQ(x.speedKmh, 36.0);
        QVERIFY(qAbs(x.steeringDeg - 30.0) < 1e-9);
        QCOMPARE(channelValue(x, TelemetryChannel::Acceleration), 1.5);
        t.clear();
        QVERIFY(t.samples().empty());
        QVERIFY(t.update(at(0.0, 0.0)));  // clock restarts after clear
    }

    void ignitionAndGearEvents() {
        EventDetector d;
        d.reset(at(0, 0), false, Gear::Drive);
        QVERIFY(d.update(at(0.1, 0), false, Gear::Drive).empty());
        QVERIFY(has(d.update(at(0.2, 0), true, Gear::Drive), "Ignition ON"));
        QVERIFY(d.update(at(0.3, 0), true, Gear::Drive).empty());  // no repeat
        QVERIFY(has(d.update(at(0.4, 0), true, Gear::Reverse), "Reverse"));
        QVERIFY(has(d.update(at(0.5, 0), false, Gear::Reverse), "Ignition OFF"));
    }

    void movingAndStoppedEvents() {
        EventDetector d;
        d.reset(at(0, 0), true, Gear::Drive);
        QVERIFY(has(d.update(at(1, 2.0), true, Gear::Drive), "moving"));
        QVERIFY(d.update(at(2, 3.0), true, Gear::Drive).empty());
        QVERIFY(has(d.update(at(3, 0.0), true, Gear::Drive), "stopped"));
    }

    void hardBrakingFiresOnceWithHysteresis() {
        EventDetector d;
        d.reset(at(0, 20), true, Gear::Drive);
        int warnings = 0;
        for (int i = 0; i < 10; ++i)
            for (const Event& e : d.update(at(i * 0.1, 20, -8.0), true, Gear::Drive))
                warnings += e.severity == Severity::Warning;
        QCOMPARE(warnings, 1);
        d.update(at(2, 20, -0.5), true, Gear::Drive);  // released
        QVERIFY(has(d.update(at(3, 20, -8.0), true, Gear::Drive), "Hard braking"));  // re-armed
    }

    void normalBrakingAndLowSpeedDoNotWarn() {
        EventDetector d;
        d.reset(at(0, 20), true, Gear::Drive);
        QVERIFY(d.update(at(1, 20, -3.0), true, Gear::Drive).empty());
        QVERIFY(d.update(at(2, 0.5, -9.0), true, Gear::Drive).size() <= 1);  // only "moving"/none, no warning
        for (const Event& e : d.update(at(3, 0.5, -9.0), true, Gear::Drive)) QVERIFY(e.severity == Severity::Info);
    }

    void highLateralAcceleration() {
        EventDetector d;
        d.reset(at(0, 20), true, Gear::Drive);
        QVERIFY(has(d.update(at(1, 20, 0, 5.0), true, Gear::Drive), "lateral"));
        QVERIFY(d.update(at(2, 20, 0, 5.0), true, Gear::Drive).empty());
    }
};

QTEST_APPLESS_MAIN(TelemetryTest)
#include "tst_telemetry.moc"
