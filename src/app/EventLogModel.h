#pragma once

#include <QAbstractListModel>
#include <QString>
#include <QVector>
#include <QtQml/qqmlregistration.h>

#include "EventDetector.h"

// List model for the on-screen event log. Capped so a long session cannot grow without bound.
class EventLogModel : public QAbstractListModel {
    Q_OBJECT
    QML_ELEMENT
    QML_UNCREATABLE("Owned by SimulationController")
    Q_PROPERTY(int count READ rowCount NOTIFY countChanged)

public:
    enum Role { TimeRole = Qt::UserRole + 1, WarningRole, MessageRole };
    static constexpr int kMaxEntries = 500;

    explicit EventLogModel(QObject* parent = nullptr) : QAbstractListModel(parent) {}

    int rowCount(const QModelIndex& parent = {}) const override { return parent.isValid() ? 0 : entries_.size(); }
    QVariant data(const QModelIndex& index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    void append(const driveforge::Event& event);
    void append(double time, driveforge::Severity severity, const QString& message);
    void clear();

signals:
    void countChanged();

private:
    struct Entry {
        double time;
        bool warning;
        QString message;
    };
    QVector<Entry> entries_;
};
