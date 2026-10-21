#include "BayModel.h"

#include <algorithm>

QVariant BayModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= bays_.size()) return {};
    const driveforge::ParkingSlot& b = bays_.at(index.row());
    switch (role) {
    case IdRole: return b.id;
    case XRole: return b.x;
    case YRole: return b.y;
    case WidthRole: return b.width;
    case LengthRole: return b.length;
    case StateRole: return states_.at(index.row());
    }
    return {};
}

QHash<int, QByteArray> BayModel::roleNames() const {
    return {{IdRole, "bayId"}, {XRole, "posX"},     {YRole, "posY"},
            {WidthRole, "width"}, {LengthRole, "length"}, {StateRole, "state"}};
}

void BayModel::sync(const driveforge::World& world, const driveforge::SensorReadings& sensors, int targetId) {
    if (world.baysVersion() != version_) {
        beginResetModel();
        bays_ = QVector<driveforge::ParkingSlot>(world.bays().begin(), world.bays().end());
        states_ = QVector<int>(bays_.size(), 0);
        version_ = world.baysVersion();
        if (!bays_.isEmpty()) {
            minY_ = bays_.first().y - bays_.first().width / 2.0;
            maxY_ = bays_.last().y + bays_.last().width / 2.0;
            x_ = bays_.first().x;
            depth_ = bays_.first().length;
        }
        endResetModel();
        emit layoutChanged();
    }
    if (bays_.isEmpty()) return;

    QVector<int> next(bays_.size(), 0);
    for (const driveforge::SlotReading& r : sensors.bays)
        for (int i = 0; i < bays_.size(); ++i)
            if (bays_[i].id == r.id && r.free) next[i] = 1;
    for (int i = 0; i < bays_.size(); ++i)
        if (bays_[i].id == targetId) next[i] = 2;
    if (next != states_) {
        states_ = next;
        emit dataChanged(index(0), index(bays_.size() - 1), {StateRole});
    }
}
