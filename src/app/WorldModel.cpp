#include "WorldModel.h"

QVariant WorldModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= objects_.size()) return {};
    const driveforge::WorldObject& o = objects_.at(index.row());
    switch (role) {
    case IdRole: return o.id;
    case KindRole: return static_cast<int>(o.type);  // 0 vehicle, 1 obstacle, 2 pedestrian
    case XRole: return o.x;
    case YRole: return o.y;
    case WidthRole: return o.width;
    case LengthRole: return o.length;
    case VxRole: return o.vx;
    case VyRole: return o.vy;
    case ActiveRole: return o.active;
    }
    return {};
}

QHash<int, QByteArray> WorldModel::roleNames() const {
    return {{IdRole, "objId"}, {KindRole, "kind"}, {XRole, "posX"},
            {YRole, "posY"},   {WidthRole, "width"}, {LengthRole, "length"},
            {VxRole, "velX"},  {VyRole, "velY"},    {ActiveRole, "moving"}};
}

void WorldModel::sync(const driveforge::World& world) {
    if (world.version() != version_) {
        beginResetModel();
        objects_ = QVector<driveforge::WorldObject>(world.objects().begin(), world.objects().end());
        version_ = world.version();
        endResetModel();
        return;
    }
    if (objects_.isEmpty()) return;
    for (int i = 0; i < objects_.size(); ++i) objects_[i] = world.objects()[static_cast<std::size_t>(i)];
    emit dataChanged(index(0), index(objects_.size() - 1), {XRole, YRole, VxRole, VyRole, ActiveRole});
}
