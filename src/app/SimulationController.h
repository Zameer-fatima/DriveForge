#pragma once

#include <QElapsedTimer>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QtCharts/QAbstractSeries>
#include <QtQml/qqmlregistration.h>

#include <QList>
#include <QPointF>

#include "BayModel.h"
#include "EventLogModel.h"
#include "ScenarioManager.h"
#include "Simulation.h"
#include "WorldModel.h"

// Qt facade over driveforge::Simulation. Owns the fixed-step loop; QML only reads state and sends commands.
class SimulationController : public QObject {
    Q_OBJECT
    QML_ELEMENT

    Q_PROPERTY(bool running READ running NOTIFY runningChanged)
    Q_PROPERTY(bool ignition READ ignition NOTIFY stateChanged)
    Q_PROPERTY(bool reverse READ reverse NOTIFY stateChanged)
    Q_PROPERTY(bool reverseRequested READ reverseRequested NOTIFY stateChanged)
    Q_PROPERTY(EventLogModel* events READ events CONSTANT)
    Q_PROPERTY(WorldModel* objects READ objects CONSTANT)
    Q_PROPERTY(BayModel* bays READ bays CONSTANT)
    Q_PROPERTY(ScenarioManager* scenarios READ scenarios CONSTANT)
    Q_PROPERTY(double simTime READ simTime NOTIFY stateChanged)
    Q_PROPERTY(double posX READ posX NOTIFY stateChanged)
    Q_PROPERTY(double posY READ posY NOTIFY stateChanged)
    Q_PROPERTY(double headingDeg READ headingDeg NOTIFY stateChanged)
    Q_PROPERTY(double speed READ speed NOTIFY stateChanged)
    Q_PROPERTY(double speedKmh READ speedKmh NOTIFY stateChanged)
    Q_PROPERTY(double acceleration READ acceleration NOTIFY stateChanged)
    Q_PROPERTY(double lateralAcceleration READ lateralAcceleration NOTIFY stateChanged)
    Q_PROPERTY(int laneChangeDir READ laneChangeDir NOTIFY stateChanged)
    Q_PROPERTY(double steeringDeg READ steeringDeg NOTIFY stateChanged)
    Q_PROPERTY(double throttle READ throttle NOTIFY stateChanged)
    Q_PROPERTY(double brake READ brake NOTIFY stateChanged)
    Q_PROPERTY(double brakingDistance READ brakingDistance NOTIFY stateChanged)
    Q_PROPERTY(double wheelSpinDeg READ wheelSpinDeg NOTIFY stateChanged)
    Q_PROPERTY(double wheelbase READ wheelbase CONSTANT)
    Q_PROPERTY(double vehicleLength READ vehicleLength CONSTANT)
    Q_PROPERTY(double vehicleWidth READ vehicleWidth CONSTANT)
    Q_PROPERTY(double wheelRadius READ wheelRadius CONSTANT)

    // Sensors (simulated)
    Q_PROPERTY(double frontGap READ frontGap NOTIFY stateChanged)
    Q_PROPERTY(bool frontTarget READ frontTarget NOTIFY stateChanged)
    Q_PROPERTY(QString radarStatus READ radarStatus NOTIFY stateChanged)
    Q_PROPERTY(QString cameraStatus READ cameraStatus NOTIFY stateChanged)
    Q_PROPERTY(int collisions READ collisions NOTIFY stateChanged)

    // ADAS
    Q_PROPERTY(bool autonomous READ autonomous NOTIFY stateChanged)
    Q_PROPERTY(bool overtakeEnabled READ overtakeEnabled NOTIFY stateChanged)
    Q_PROPERTY(QString overtakeText READ overtakeText NOTIFY stateChanged)
    Q_PROPERTY(bool aebArmed READ aebArmed NOTIFY stateChanged)
    Q_PROPERTY(bool aebWarning READ aebWarning NOTIFY stateChanged)
    Q_PROPERTY(bool aebBraking READ aebBraking NOTIFY stateChanged)
    Q_PROPERTY(bool accEnabled READ accEnabled NOTIFY stateChanged)
    Q_PROPERTY(double accSetSpeedKmh READ accSetSpeedKmh NOTIFY stateChanged)
    Q_PROPERTY(double accTimeGap READ accTimeGap NOTIFY stateChanged)
    Q_PROPERTY(bool accHasLead READ accHasLead NOTIFY stateChanged)
    Q_PROPERTY(bool lkaEnabled READ lkaEnabled NOTIFY stateChanged)
    Q_PROPERTY(bool lkaOverridden READ lkaOverridden NOTIFY stateChanged)
    Q_PROPERTY(bool laneChanging READ laneChanging NOTIFY stateChanged)
    Q_PROPERTY(double laneOffset READ laneOffset NOTIFY stateChanged)

    // Parking assist
    Q_PROPERTY(bool parkingEnabled READ parkingEnabled NOTIFY stateChanged)
    Q_PROPERTY(bool parkingReady READ parkingReady NOTIFY stateChanged)
    Q_PROPERTY(bool parkingActive READ parkingActive NOTIFY stateChanged)
    Q_PROPERTY(bool parked READ parked NOTIFY stateChanged)
    Q_PROPERTY(QString parkingText READ parkingText NOTIFY stateChanged)
    Q_PROPERTY(QList<QPointF> parkingPath READ parkingPath NOTIFY parkingPathChanged)

public:
    explicit SimulationController(QObject* parent = nullptr);

    bool running() const { return running_; }
    bool ignition() const { return sim_.driver().ignition; }
    bool reverse() const { return sim_.vehicle().gear() == driveforge::Gear::Reverse; }
    bool reverseRequested() const { return sim_.driver().gear == driveforge::Gear::Reverse; }
    EventLogModel* events() { return &events_; }
    WorldModel* objects() { return &worldModel_; }
    BayModel* bays() { return &bayModel_; }
    ScenarioManager* scenarios() { return &scenarios_; }
    double simTime() const { return state().time; }
    double posX() const { return state().x; }
    double posY() const { return state().y; }
    double headingDeg() const;
    double speed() const { return state().speed; }
    double speedKmh() const { return state().speed * 3.6; }
    double acceleration() const { return displayAccel_; }
    double lateralAcceleration() const { return displayLateral_; }
    int laneChangeDir() const { return sim_.adas().status().laneChangeDir; }
    double steeringDeg() const;
    double throttle() const { return state().throttle; }
    double brake() const { return state().brake; }
    double brakingDistance() const { return sim_.vehicle().brakingDistance(std::abs(state().speed)); }
    double wheelSpinDeg() const;
    double wheelbase() const { return sim_.vehicle().params().wheelbase; }
    double vehicleLength() const { return sim_.vehicle().params().length; }
    double vehicleWidth() const { return sim_.vehicle().params().width; }
    double wheelRadius() const { return sim_.vehicle().params().wheelRadius; }

    double frontGap() const { return sim_.frontGap(); }
    bool frontTarget() const { return sim_.frontGap() < sim_.sensors().frontRange; }
    QString radarStatus() const;
    QString cameraStatus() const;
    int collisions() const { return sim_.collisions(); }

    bool autonomous() const { return sim_.adas().status().mode == driveforge::DriveMode::Autonomous; }
    bool overtakeEnabled() const { return sim_.adas().status().overtakeEnabled; }
    QString overtakeText() const { return QString::fromStdString(sim_.adas().status().overtakeText); }
    bool aebArmed() const { return sim_.adas().status().aebArmed; }
    bool aebWarning() const { return sim_.adas().status().aebWarning; }
    bool aebBraking() const { return sim_.adas().status().aebBraking; }
    bool accEnabled() const { return sim_.adas().status().accEnabled; }
    double accSetSpeedKmh() const { return sim_.adas().status().accSetSpeed * 3.6; }
    double accTimeGap() const { return sim_.adas().status().accTimeGap; }
    bool accHasLead() const { return sim_.adas().status().accHasLead; }
    bool lkaEnabled() const { return sim_.adas().status().lkaEnabled; }
    bool lkaOverridden() const { return sim_.adas().status().lkaOverridden; }
    bool laneChanging() const { return sim_.adas().status().laneChanging; }
    double laneOffset() const { return sim_.adas().status().lkaOffset; }

    bool parkingEnabled() const { return sim_.adas().status().parkingEnabled; }
    bool parkingReady() const { return sim_.adas().status().parkingPhase == driveforge::ParkingPhase::Ready; }
    bool parkingActive() const { return sim_.adas().status().parkingActive; }
    bool parked() const { return sim_.adas().status().parkingPhase == driveforge::ParkingPhase::Parked; }
    QString parkingText() const { return QString::fromStdString(sim_.adas().parking().statusText()); }
    QList<QPointF> parkingPath() const { return parkingPath_; }

    // Driver inputs (continuous values, set from QML key handling).
    Q_INVOKABLE void setDriverInput(double throttle, double brake, double steer);
    Q_INVOKABLE void toggleIgnition();
    Q_INVOKABLE void toggleGear();
    Q_INVOKABLE void togglePause();
    Q_INVOKABLE void reset();

    // ADAS commands. They are queued and handled on the next simulation step.
    Q_INVOKABLE void toggleAutonomous();
    Q_INVOKABLE void toggleAcc();
    Q_INVOKABLE void toggleLka();
    Q_INVOKABLE void toggleAeb();
    Q_INVOKABLE void toggleOvertake();
    Q_INVOKABLE void laneChange(int direction);  // -1 left, +1 right
    Q_INVOKABLE void accSpeed(int direction);    // -1 / +1 (5 km/h)
    Q_INVOKABLE void nextTimeGap();
    Q_INVOKABLE void toggleParkingAssist();
    Q_INVOKABLE void startParking();

    // Test objects, placed ahead of the car (simulated).
    Q_INVOKABLE void spawnObstacle();
    Q_INVOKABLE void spawnPedestrian();
    Q_INVOKABLE void spawnLeadCar(double speedKmh = 50.0);
    Q_INVOKABLE void spawnParkingLot();
    Q_INVOKABLE void clearObjects();

    // Replace a Qt Charts XY series with a telemetry channel (see driveforge::TelemetryChannel).
    Q_INVOKABLE void fillSeries(QAbstractSeries* series, int channel) const;

signals:
    void runningChanged();
    void stateChanged();
    void telemetryUpdated();
    void parkingPathChanged();
    // A warning was logged; QML shows it as a banner so it cannot be missed.
    void noticePosted(const QString& text);

private:
    const driveforge::VehicleState& state() const { return sim_.vehicle().state(); }
    void tick();
    void drainEvents();
    bool lockedByScenario();  // true (and warns) while a scenario drives the car
    void syncScene();
    void log(driveforge::Severity severity, const QString& message);

    driveforge::Simulation sim_;
    ScenarioManager scenarios_{sim_};
    EventLogModel events_;
    WorldModel worldModel_;
    BayModel bayModel_;
    QList<QPointF> parkingPath_;
    int planRevision_ = -1;
    QTimer timer_;
    QElapsedTimer clock_;
    qint64 lastNs_ = 0;
    double accumulator_ = 0.0;
    double displayAccel_ = 0.0;
    double displayLateral_ = 0.0;
    bool running_ = true;
};
