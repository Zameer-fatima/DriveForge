#include "EventLogModel.h"

QVariant EventLogModel::data(const QModelIndex& index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= entries_.size()) return {};
    const Entry& e = entries_.at(index.row());
    switch (role) {
    case TimeRole: return QString::asprintf("%6.1f s", e.time);
    case WarningRole: return e.warning;
    case MessageRole: return e.message;
    }
    return {};
}

QHash<int, QByteArray> EventLogModel::roleNames() const {
    return {{TimeRole, "time"}, {WarningRole, "warning"}, {MessageRole, "message"}};
}

void EventLogModel::append(double time, driveforge::Severity severity, const QString& message) {
    if (entries_.size() >= kMaxEntries) {
        beginRemoveRows({}, 0, 0);
        entries_.removeFirst();
        endRemoveRows();
    }
    beginInsertRows({}, entries_.size(), entries_.size());
    entries_.push_back({time, severity == driveforge::Severity::Warning, message});
    endInsertRows();
    emit countChanged();
}

void EventLogModel::append(const driveforge::Event& event) {
    append(event.time, event.severity, QString::fromStdString(event.message));
}

void EventLogModel::clear() {
    beginResetModel();
    entries_.clear();
    endResetModel();
    emit countChanged();
}
