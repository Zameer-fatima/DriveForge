#pragma once

#include <QAbstractListModel>
#include <QVector>
#include <QtQml/qqmlregistration.h>

#include "Sensors.h"
#include "World.h"

// Parking bays for the 3D scene. state: 0 = other/occupied/out of range, 1 = free (detected), 2 = the bay being parked in.
class BayModel : public QAbstractListModel {
    Q_OBJECT
    QML_ELEMENT
    QML_UNCREATABLE("Owned by SimulationController")
    Q_PROPERTY(bool hasBays READ hasBays NOTIFY layoutChanged)
    Q_PROPERTY(double lotMinY READ lotMinY NOTIFY layoutChanged)
    Q_PROPERTY(double lotMaxY READ lotMaxY NOTIFY layoutChanged)
    Q_PROPERTY(double lotX READ lotX NOTIFY layoutChanged)
    Q_PROPERTY(double lotDepth READ lotDepth NOTIFY layoutChanged)

public:
    enum Role { IdRole = Qt::UserRole + 1, XRole, YRole, WidthRole, LengthRole, StateRole };

    explicit BayModel(QObject* parent = nullptr) : QAbstractListModel(parent) {}

    int rowCount(const QModelIndex& parent = {}) const override { return parent.isValid() ? 0 : bays_.size(); }
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    bool hasBays() const { return !bays_.isEmpty(); }
    double lotMinY() const { return minY_; }
    double lotMaxY() const { return maxY_; }
    double lotX() const { return x_; }
    double lotDepth() const { return depth_; }

    // targetId: the bay being parked in (or -1).
    void sync(const driveforge::World& world, const driveforge::SensorReadings& sensors, int targetId);

signals:
    void layoutChanged();

private:
    QVector<driveforge::ParkingSlot> bays_;
    QVector<int> states_;
    int version_ = -1;
    double minY_ = 0.0, maxY_ = 0.0, x_ = 0.0, depth_ = 0.0;
};
