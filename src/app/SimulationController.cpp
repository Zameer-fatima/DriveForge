#include "SimulationController.h"

#include <algorithm>
#include <cmath>

#include <QPointF>
#include <QtCharts/QXYSeries>

namespace {
constexpr double kStep = 1.0 / 120.0;
constexpr double kMaxFrame = 0.1;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
}  // namespace

using driveforge::AdasRequest;
using driveforge::Severity;

SimulationController::SimulationController(QObject* parent) : QObject(parent) {
    log(Severity::Info, "Simulation ready (all data simulated)");
    connect(&scenarios_, &ScenarioManager::liveStarted, this, [this](const QString& name) {
        events_.clear();
        log(Severity::Info, "Scenario started: " + name + " (controls locked, Esc aborts)");
        syncScene();
        accumulator_ = 0.0;
        displayAccel_ = 0.0;
        lastNs_ = clock_.nsecsElapsed();
        running_ = true;
        emit runningChanged();
        emit stateChanged();
        emit telemetryUpdated();
    });
    connect(&scenarios_, &ScenarioManager::finished, this, [this](const QString& name, const QString& outcome, const QString& detail) {
        log(outcome == "PASS" ? Severity::Info : Severity::Warning, QString("Scenario %1: %2 (%3)").arg(name, outcome, detail));
    });
    timer_.setTimerType(Qt::PreciseTimer);
    timer_.setInterval(8);
    connect(&timer_, &QTimer::timeout, this, &SimulationController::tick);
    clock_.start();
    lastNs_ = clock_.nsecsElapsed();
    timer_.start();
}

double SimulationController::headingDeg() const { return state().heading * kRadToDeg; }
double SimulationController::steeringDeg() const { return state().steerAngle * kRadToDeg; }
double SimulationController::wheelSpinDeg() const { return state().travel / wheelRadius() * kRadToDeg; }

QString SimulationController::radarStatus() const {
    if (sim_.sensors().detections.empty()) return QStringLiteral("clear");
    if (sim_.frontGap() < sim_.sensors().frontRange)
        return QStringLiteral("tracking target in lane");
    return QStringLiteral("objects detected (not in lane)");
}

QString SimulationController::cameraStatus() const {
    return sim_.sensors().cameraOk ? QStringLiteral("lane markings OK") : QStringLiteral("no lane markings (off the road)");
}

void SimulationController::log(Severity severity, const QString& message) {
    events_.append(state().time, severity, message);
    if (severity == Severity::Warning) emit noticePosted(message);
}

void SimulationController::syncScene() {
    worldModel_.sync(sim_.world());
    bayModel_.sync(sim_.world(), sim_.sensors(),
                   sim_.adas().status().parkingEnabled ? sim_.adas().parking().slotId() : -1);
    const int revision = sim_.adas().parking().planRevision();
    if (revision != planRevision_) {
        planRevision_ = revision;
        parkingPath_.clear();
        const auto& plan = sim_.adas().parking().plan();
        if (plan.valid) {
            // Start of the arc, then the planned reverse trajectory.
            parkingPath_.append(QPointF(plan.x0, plan.y0));
            for (const auto& pose : plan.path) parkingPath_.append(QPointF(pose.x, pose.y));
        }
        emit parkingPathChanged();
    }
}

void SimulationController::drainEvents() {
    for (const driveforge::Event& e : sim_.takeEvents()) {
        events_.append(e);
        if (e.severity == Severity::Warning) emit noticePosted(QString::fromStdString(e.message));
    }
}

bool SimulationController::lockedByScenario() {
    if (!scenarios_.running()) return false;
    emit noticePosted("Controls are locked while a scenario runs. Press Esc to abort it.");
    return true;
}

void SimulationController::setDriverInput(double throttle, double brake, double steer) {
    if (scenarios_.running()) return;  // scripted driver
    sim_.driver().throttle = throttle;
    sim_.driver().brake = brake;
    sim_.driver().steer = steer;
}

void SimulationController::toggleIgnition() {
    if (lockedByScenario()) return;
    sim_.driver().ignition = !sim_.driver().ignition;
    emit stateChanged();
}

void SimulationController::toggleGear() {
    if (lockedByScenario()) return;
    // The model only honours the change once the car is stationary.
    auto& g = sim_.driver().gear;
    g = (g == driveforge::Gear::Drive) ? driveforge::Gear::Reverse : driveforge::Gear::Drive;
    emit stateChanged();
}

void SimulationController::togglePause() {
    running_ = !running_;
    log(Severity::Info, running_ ? "Resumed" : "Paused");
    lastNs_ = clock_.nsecsElapsed();
    emit runningChanged();
}

void SimulationController::reset() {
    if (scenarios_.running()) scenarios_.abort("simulation reset");
    sim_.reset();
    events_.clear();
    log(Severity::Info, "Simulation reset");
    syncScene();
    accumulator_ = 0.0;
    displayAccel_ = 0.0;
    displayLateral_ = 0.0;
    lastNs_ = clock_.nsecsElapsed();
    emit stateChanged();
    emit telemetryUpdated();
}

void SimulationController::toggleAutonomous() {
    if (!lockedByScenario()) sim_.adas().request(AdasRequest::ToggleAutonomous);
}
void SimulationController::toggleAcc() {
    if (!lockedByScenario()) sim_.adas().request(AdasRequest::ToggleAcc);
}
void SimulationController::toggleLka() {
    if (!lockedByScenario()) sim_.adas().request(AdasRequest::ToggleLka);
}
void SimulationController::toggleAeb() {
    if (!lockedByScenario()) sim_.adas().request(AdasRequest::ToggleAeb);
}
void SimulationController::toggleOvertake() {
    if (!lockedByScenario()) sim_.adas().request(AdasRequest::ToggleOvertake);
}
void SimulationController::nextTimeGap() {
    if (!lockedByScenario()) sim_.adas().request(AdasRequest::AccNextTimeGap);
}
void SimulationController::toggleParkingAssist() {
    if (!lockedByScenario()) sim_.adas().request(AdasRequest::ToggleParkingAssist);
}
void SimulationController::startParking() {
    if (!lockedByScenario()) sim_.adas().request(AdasRequest::StartParking);
}
void SimulationController::laneChange(int direction) {
    if (lockedByScenario()) return;
    sim_.adas().request(direction < 0 ? AdasRequest::LaneChangeLeft : AdasRequest::LaneChangeRight);
}
void SimulationController::accSpeed(int direction) {
    if (lockedByScenario()) return;
    sim_.adas().request(direction < 0 ? AdasRequest::AccSpeedDown : AdasRequest::AccSpeedUp);
}

void SimulationController::spawnObstacle() {
    if (lockedByScenario()) return;
    sim_.spawnObstacle(120.0);
    log(Severity::Info, "Spawned obstacle 120 m ahead");
    syncScene();
}

void SimulationController::spawnPedestrian() {
    if (lockedByScenario()) return;
    sim_.spawnPedestrian(std::max(80.0, std::abs(state().speed) * 7.0));
    log(Severity::Info, "Spawned pedestrian ahead (crosses from the right)");
    syncScene();
}

void SimulationController::spawnLeadCar(double speedKmh) {
    if (lockedByScenario()) return;
    sim_.spawnLeadVehicle(60.0, speedKmh / 3.6);
    log(Severity::Info, QString::asprintf("Spawned lead car 60 m ahead at %.0f km/h", speedKmh));
    syncScene();
}

void SimulationController::spawnParkingLot() {
    if (lockedByScenario()) return;
    // First free bay about 3 s ahead, always inside the 40 m slot sensor range.
    const double ahead = std::clamp(std::abs(state().speed) * 3.0 + 10.0, 10.0, 38.0);
    sim_.spawnParkingLot(ahead);
    log(Severity::Info, QString::asprintf("Parking lot added on the RIGHT side, first free bay %.0f m ahead. Press N", ahead));
    if (std::abs(state().speed) > 11.0)
        log(Severity::Warning, "Slow down below 40 km/h to park; the assist can also reverse back to the bay");
    syncScene();
}

void SimulationController::clearObjects() {
    if (lockedByScenario()) return;
    sim_.world().clear();
    sim_.world().clearBays();
    log(Severity::Info, "Removed all objects and the parking lot");
    syncScene();
}

void SimulationController::tick() {
    const qint64 now = clock_.nsecsElapsed();
    const double frame = std::min(kMaxFrame, static_cast<double>(now - lastNs_) * 1e-9);
    lastNs_ = now;
    if (!running_) return;

    accumulator_ += frame * (scenarios_.running() ? scenarios_.timeScale() : 1.0);
    bool sampled = false;
    while (accumulator_ >= kStep) {
        scenarios_.beforeStep();
        sim_.step(kStep);
        scenarios_.afterStep();
        displayAccel_ += (state().acceleration - displayAccel_) * 0.05;  // low-pass for the UI only
        displayLateral_ += (state().lateralAcceleration - displayLateral_) * 0.05;
        sampled |= sim_.takeTelemetryFlag();
        accumulator_ -= kStep;
    }
    drainEvents();
    syncScene();
    emit stateChanged();
    if (sampled) emit telemetryUpdated();
}

void SimulationController::fillSeries(QAbstractSeries* series, int channel) const {
    auto* xy = qobject_cast<QXYSeries*>(series);
    if (!xy) return;
    const auto ch = static_cast<driveforge::TelemetryChannel>(channel);
    const auto& samples = sim_.telemetry().samples();
    QList<QPointF> points;
    points.reserve(static_cast<qsizetype>(samples.size()));
    for (std::size_t i = 0; i < samples.size(); ++i)
        points.append(QPointF(samples.at(i).time, driveforge::channelValue(samples.at(i), ch)));
    xy->replace(points);
}
