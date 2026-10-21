#pragma once

#include <QAbstractListModel>
#include <QDateTime>
#include <QObject>
#include <QString>
#include <QVariantList>
#include <map>
#include <QtQml/qqmlregistration.h>

#include "ScenarioRunner.h"

// Editable parameters of the selected scenario, for the sliders in the scenario panel.
class ScenarioParamModel : public QAbstractListModel {
    Q_OBJECT
    QML_ELEMENT
    QML_UNCREATABLE("Owned by ScenarioManager")

public:
    enum Role { KeyRole = Qt::UserRole + 1, LabelRole, UnitRole, ValueRole, MinRole, MaxRole, StepRole };

    explicit ScenarioParamModel(QObject* parent = nullptr) : QAbstractListModel(parent) {}
    int rowCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    void show(driveforge::ScenarioId id, driveforge::ParamMap* values);
    void refresh();
    bool set(int row, double value);

private:
    driveforge::ScenarioId id_ = driveforge::ScenarioId::Highway;
    driveforge::ParamMap* values_ = nullptr;
};

class ScenarioManager : public QObject {
    Q_OBJECT
    QML_ELEMENT
    QML_UNCREATABLE("Owned by SimulationController")

    Q_PROPERTY(QVariantList catalog READ catalog CONSTANT)
    Q_PROPERTY(int selected READ selected WRITE setSelected NOTIFY selectedChanged)
    Q_PROPERTY(QString description READ description NOTIFY selectedChanged)
    Q_PROPERTY(ScenarioParamModel* params READ params CONSTANT)
    Q_PROPERTY(bool running READ running NOTIFY runningChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(double timeScale READ timeScale WRITE setTimeScale NOTIFY timeScaleChanged)
    Q_PROPERTY(bool hasReport READ hasReport NOTIFY reportChanged)
    Q_PROPERTY(QString outcome READ outcome NOTIFY reportChanged)
    Q_PROPERTY(QString reportTitle READ reportTitle NOTIFY reportChanged)
    Q_PROPERTY(QString summary READ summary NOTIFY reportChanged)
    Q_PROPERTY(QVariantList criteria READ criteria NOTIFY reportChanged)
    Q_PROPERTY(QString repeatSummary READ repeatSummary NOTIFY reportChanged)
    Q_PROPERTY(QString exportPath READ exportPath NOTIFY reportChanged)

public:
    explicit ScenarioManager(driveforge::Simulation& sim, QObject* parent = nullptr);

    QVariantList catalog() const;
    int selected() const { return static_cast<int>(selected_); }
    void setSelected(int index);
    QString description() const;
    ScenarioParamModel* params() { return &paramModel_; }
    bool running() const { return runner_.running(); }
    QString status() const;
    double timeScale() const { return timeScale_; }
    void setTimeScale(double scale);
    bool hasReport() const { return hasReport_; }
    QString outcome() const { return hasReport_ ? QString::fromStdString(report_.outcomeText()) : QString(); }
    QString reportTitle() const { return hasReport_ ? QString::fromStdString(report_.name) : QString(); }
    QString summary() const;
    QVariantList criteria() const;
    QString repeatSummary() const { return repeatSummary_; }
    QString exportPath() const { return exportPath_; }

    Q_INVOKABLE void setParam(int row, double value);
    Q_INVOKABLE void resetParams();
    Q_INVOKABLE void runLive();
    Q_INVOKABLE void abort(const QString& reason = QStringLiteral("stopped by user"));
    // Runs the selected scenario `times` times headless and compares the results exactly.
    Q_INVOKABLE void runRepeatCheck(int times = 3);
    Q_INVOKABLE QString exportReport();

    // Called by the simulation loop around every step.
    void beforeStep() { runner_.beforeStep(sim_); }
    void afterStep();

signals:
    void selectedChanged();
    void runningChanged();
    void statusChanged();
    void timeScaleChanged();
    void reportChanged();
    void liveStarted(const QString& name);
    void finished(const QString& name, const QString& outcome, const QString& detail);

private:
    driveforge::ScenarioId selectedId() const { return selected_; }
    void takeReport(const driveforge::ScenarioReport& report);

    driveforge::Simulation& sim_;
    driveforge::ScenarioRunner runner_;
    driveforge::ScenarioId selected_ = driveforge::ScenarioId::Highway;
    std::map<driveforge::ScenarioId, driveforge::ParamMap> params_;
    ScenarioParamModel paramModel_;
    driveforge::ScenarioReport report_;
    bool hasReport_ = false;
    bool wasRunning_ = false;
    double timeScale_ = 1.0;
    QString repeatSummary_;
    QString exportPath_;
    QDateTime reportTime_;
};
