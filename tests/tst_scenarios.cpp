#include <QtTest>

#include "ScenarioRunner.h"

using namespace driveforge;

namespace {
bool contains(const std::string& text, const char* part) { return text.find(part) != std::string::npos; }

const Criterion* criterion(const ScenarioReport& r, const char* name) {
    for (const Criterion& c : r.criteria)
        if (c.name == name) return &c;
    return nullptr;
}
}  // namespace

class ScenarioTest : public QObject {
    Q_OBJECT
private slots:
    void catalogIsComplete() {
        QCOMPARE(scenarioCatalog().size(), size_t(4));
        for (const ScenarioInfo& info : scenarioCatalog()) {
            QVERIFY(!info.name.empty());
            QVERIFY(!info.params.empty());
            ScenarioId id;
            QVERIFY(scenarioIdFromKey(info.key, &id));
            QCOMPARE(int(id), int(info.id));
            for (const ParamSpec& p : info.params) {
                QVERIFY2(p.min <= p.def && p.def <= p.max, qPrintable(QString::fromStdString(info.key + "." + p.key)));
                QVERIFY(p.step > 0.0);
            }
        }
        ScenarioId unused;
        QVERIFY(!scenarioIdFromKey("nonsense", &unused));
    }

    void resolveParamsClampsAndFillsDefaults() {
        const ParamMap m = resolveParams(ScenarioId::Obstacle, {{"speed", 500.0}, {"distance", -5.0}, {"bogus", 1.0}});
        QCOMPARE(m.at("speed"), 130.0);
        QCOMPARE(m.at("distance"), 40.0);
        QCOMPARE(m.at("driverThrottle"), 1.0);  // default
        QVERIFY(m.find("bogus") == m.end());
    }

    void everyScenarioPassesWithDefaults() {
        for (const ScenarioInfo& info : scenarioCatalog()) {
            const ScenarioReport r = ScenarioRunner::runHeadless(info.id);
            QVERIFY2(r.passed(), qPrintable(QString::fromStdString(info.key + ": " + r.toMarkdown())));
            QVERIFY(!r.criteria.empty());
            QVERIFY(r.duration > 1.0);
        }
    }

    void runsAreRepeatable() {
        for (const ScenarioInfo& info : scenarioCatalog()) {
            const ScenarioReport a = ScenarioRunner::runHeadless(info.id);
            const ScenarioReport b = ScenarioRunner::runHeadless(info.id);
            const ScenarioReport c = ScenarioRunner::runHeadless(info.id);
            QVERIFY2(a.sameResultAs(b) && b.sameResultAs(c), qPrintable(QString::fromStdString(info.key)));
            QCOMPARE(a.toJson(), b.toJson());
        }
    }

    void differentParametersGiveDifferentResults() {
        const ScenarioReport a = ScenarioRunner::runHeadless(ScenarioId::Obstacle, {{"speed", 60.0}});
        const ScenarioReport b = ScenarioRunner::runHeadless(ScenarioId::Obstacle, {{"speed", 100.0}});
        QVERIFY(!a.sameResultAs(b));
    }

    void reportContents() {
        const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Pedestrian);
        const std::string md = r.toMarkdown();
        QVERIFY(contains(md, "Pedestrian crossing"));
        QVERIFY(contains(md, "**Result: PASS**"));
        QVERIFY(contains(md, "SIMULATED"));
        QVERIFY(contains(md, "Start speed"));
        QVERIFY(contains(md, "No collision"));
        QVERIFY(contains(md, "not a safety-certified"));
        const std::string js = r.toJson();
        QVERIFY(contains(js, "\"result\": \"PASS\""));
        QVERIFY(contains(js, "\"scenario\": \"pedestrian\""));
        QVERIFY(contains(js, "\"data_source\": \"simulated\""));
    }

    // ---- failures are detected (the point of a test bench) ----
    void obstacleWithAebOffFails() {
        const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Obstacle, {{"aebArmed", 0.0}});
        QCOMPARE(int(r.outcome), int(Outcome::Fail));
        QCOMPARE(r.endReason, std::string("collision"));
        const Criterion* c = criterion(r, "No collision");
        QVERIFY(c && !c->passed);
        QVERIFY(contains(c->measured, "1 collision"));
        QVERIFY(contains(r.toMarkdown(), "**Result: FAIL**"));
    }

    void pedestrianWithoutAebFails() {
        const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Pedestrian, {{"aebArmed", 0.0}, {"autonomous", 0.0}});
        QCOMPARE(int(r.outcome), int(Outcome::Fail));
        QCOMPARE(r.metrics.at("collisions"), 1.0);
    }

    void unavoidableHighwayCollisionFails() {
        // 130 km/h into a 30 km/h car 40 m ahead needs ~9.6 m/s²: physically impossible to avoid.
        const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Highway, {{"egoSpeed", 130.0}, {"leadSpeed", 30.0}, {"gap", 40.0}});
        QCOMPARE(int(r.outcome), int(Outcome::Fail));
    }

    // ---- behaviour across parameter ranges ----
    void obstacleIsAvoidedAtAllSpeeds() {
        for (double kmh : {30.0, 60.0, 100.0, 130.0}) {
            const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Obstacle, {{"speed", kmh}});
            QVERIFY2(r.passed(), qPrintable(QString("speed %1").arg(kmh)));
            QCOMPARE(r.metrics.at("aeb_braked"), 1.0);
        }
    }

    void obstacleInOtherLaneCausesNoFalseBraking() {
        const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Obstacle, {{"obstacleLane", 2.0}});
        QVERIFY(r.passed());
        QCOMPARE(r.metrics.at("aeb_braked"), 0.0);
        QCOMPARE(r.endReason, std::string("obstacle passed"));
    }

    void pedestrianIsAvoidedAcrossSpeeds() {
        for (double kmh : {30.0, 50.0, 70.0}) {
            const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Pedestrian, {{"speed", kmh}});
            QVERIFY2(r.passed(), qPrintable(QString("speed %1: %2").arg(kmh).arg(QString::fromStdString(r.toMarkdown()))));
            QCOMPARE(r.metrics.at("aeb_braked"), 1.0);  // the pedestrian is in the path on arrival, AEB must act
        }
    }

    void pedestrianSteppingIntoTheFlankIsAvoided() {
        // Regression: AEB used to check only where the pedestrian is when the front bumper arrives.
        for (double ttc : {4.0, 4.5, 5.0}) {
            const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Pedestrian, {{"triggerTtc", ttc}});
            QCOMPARE(r.metrics.at("collisions"), 0.0);
        }
    }

    void highwayKeepsDistanceAndLane() {
        const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Highway);
        QVERIFY(r.passed());
        QVERIFY(r.metrics.at("min_separation_m") >= 2.0);
        QVERIFY(r.metrics.at("max_lane_offset_m") <= 0.5);
        QCOMPARE(r.endReason, std::string("duration reached"));
    }

    void parkingVariantsPass() {
        for (double variant : {1.0, 2.0, 3.0}) {
            const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Parking, {{"freeBays", variant}});
            QVERIFY2(r.passed(), qPrintable(QString("variant %1").arg(variant)));
            QVERIFY(r.metrics.at("final_offset_m") <= 0.4);
            QVERIFY(r.metrics.at("final_heading_error_deg") <= 5.0);
        }
    }

    void parkingFromDifferentStarts() {
        for (double x : {-2.0, 2.0, 5.0}) {
            const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Parking, {{"startX", x}, {"startSpeed", 20.0}});
            QVERIFY2(r.passed(), qPrintable(QString("startX %1").arg(x)));
        }
    }

    void parkingToleranceIsAnEvaluatedCriterion() {
        // With the tightest allowed offset the run still evaluates all criteria; they are reported, pass or fail.
        const ScenarioReport r = ScenarioRunner::runHeadless(ScenarioId::Parking, {{"maxOffset", 0.2}});
        QCOMPARE(r.criteria.size(), size_t(5));
        QVERIFY(criterion(r, "Final position") != nullptr);
    }

    // ---- live use on a shared Simulation ----
    void liveRunHandsControlBack() {
        Simulation sim;
        ScenarioRunner runner;
        runner.start(sim, ScenarioId::Obstacle);
        QVERIFY(runner.running());
        QVERIFY(contains(runner.statusText(), "Obstacle ahead"));
        for (int i = 0; i < 120 * 40 && runner.running(); ++i) {
            runner.beforeStep(sim);
            sim.step(1.0 / 120.0);
            runner.afterStep(sim);
        }
        QVERIFY(runner.finished());
        QVERIFY(runner.report().passed());
        QVERIFY(!sim.adas().status().accEnabled);
        QVERIFY(sim.driver().brake > 0.0 && sim.driver().brake < 0.7);  // moderate brake, controls handed back
        QVERIFY(contains(runner.statusText(), "PASS"));
    }

    void abortProducesAbortedReport() {
        Simulation sim;
        ScenarioRunner runner;
        runner.start(sim, ScenarioId::Highway);
        for (int i = 0; i < 120 * 3; ++i) {
            runner.beforeStep(sim);
            sim.step(1.0 / 120.0);
            runner.afterStep(sim);
        }
        runner.abort(sim, "stopped by user");
        QVERIFY(runner.finished());
        QCOMPARE(int(runner.report().outcome), int(Outcome::Aborted));
        QCOMPARE(runner.report().endReason, std::string("stopped by user"));
        QVERIFY(contains(runner.report().toMarkdown(), "ABORTED"));
    }

    void startingAScenarioClearsThePreviousOne() {
        Simulation sim;
        ScenarioRunner runner;
        runner.start(sim, ScenarioId::Parking);
        QVERIFY(!sim.world().bays().empty());
        runner.start(sim, ScenarioId::Obstacle);
        QVERIFY(sim.world().bays().empty());
        QCOMPARE(sim.world().objects().size(), size_t(1));
        QCOMPARE(sim.vehicle().state().time, 0.0);
    }

    void separationIsZeroWhenTouchingAndPositiveOtherwise() {
        VehicleModel m;
        m.reset(0, 0, 0, 0);
        WorldObject o = World::makeObstacle(0, 5.0);  // ego front at 2.25, obstacle rear at 4.5
        QVERIFY(std::abs(separation(m.state(), m.params(), o) - 2.25) < 1e-9);
        o.y = 2.6;
        QCOMPARE(separation(m.state(), m.params(), o), 0.0);
        o = World::makeObstacle(3.0, 0.0);  // beside: side gap 3.0 - 0.8 - 0.925
        QVERIFY(std::abs(separation(m.state(), m.params(), o) - (3.0 - 0.8 - 0.925)) < 1e-9);
    }
};

QTEST_APPLESS_MAIN(ScenarioTest)
#include "tst_scenarios.moc"
