#pragma once

#include <QAbstractListModel>
#include <QVector>
#include <QtQml/qqmlregistration.h>

#include "World.h"

// Feeds world objects (obstacles, pedestrians, vehicles) to the 3D scene's Repeater3D.
class WorldModel : public QAbstractListModel {
    Q_OBJECT
    QML_ELEMENT
    QML_UNCREATABLE("Owned by SimulationController")

public:
    enum Role { IdRole = Qt::UserRole + 1, KindRole, XRole, YRole, WidthRole, LengthRole, VxRole, VyRole, ActiveRole };

    explicit WorldModel(QObject* parent = nullptr) : QAbstractListModel(parent) {}

    int rowCount(const QModelIndex& parent = {}) const override { return parent.isValid() ? 0 : objects_.size(); }
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    // Call after every simulation tick: resets the list when objects were added/removed, otherwise only updates poses.
    void sync(const driveforge::World& world);

private:
    QVector<driveforge::WorldObject> objects_;
    int version_ = -1;
};
