#pragma once

#include <QElapsedTimer>
#include <QList>
#include <QObject>
#include <QPointF>
#include <QQuickImageProvider>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QtCharts/QAbstractSeries>
#include <QtQml/qqmlregistration.h>
#include <memory>
#include <mutex>

#include "Recorded.h"

// The dataset currently shown, shared between RecordedData (QML side) and the image provider (render thread).
class RecordedStore {
public:
    static std::shared_ptr<const driveforge::RecordedDataset> get();
    static int serial();
    static void set(std::shared_ptr<const driveforge::RecordedDataset> data);
};

// image://recorded/camera/<serial>/<frame> and image://recorded/lidar/<serial>/<frame>. Only files listed by the loader
// are ever opened. Every image carries a burned-in "RECORDED" label.
class RecordedImageProvider : public QQuickImageProvider {
public:
    RecordedImageProvider() : QQuickImageProvider(QQuickImageProvider::Image) {}
    QImage requestImage(const QString& id, QSize* size, const QSize& requestedSize) override;
};

// QML-facing replay of recorded real-world driving data. Separate from the simulation: it never touches it.
class RecordedData : public QObject {
    Q_OBJECT
    QML_ELEMENT

    Q_PROPERTY(bool loaded READ loaded NOTIFY changed)
    Q_PROPERTY(QString error READ error NOTIFY changed)
    Q_PROPERTY(QStringList warnings READ warnings NOTIFY changed)
    Q_PROPERTY(QString name READ name NOTIFY changed)
    Q_PROPERTY(QString source READ source NOTIFY changed)
    Q_PROPERTY(QString license READ license NOTIFY changed)
    Q_PROPERTY(QString summary READ summary NOTIFY changed)
    Q_PROPERTY(int frameCount READ frameCount NOTIFY changed)
    Q_PROPERTY(double duration READ duration NOTIFY changed)
    Q_PROPERTY(bool hasCamera READ hasCamera NOTIFY changed)
    Q_PROPERTY(bool hasLidar READ hasLidar NOTIFY changed)
    Q_PROPERTY(bool hasObjects READ hasObjects NOTIFY changed)

    Q_PROPERTY(double time READ time NOTIFY frameChanged)
    Q_PROPERTY(int frame READ frame NOTIFY frameChanged)
    Q_PROPERTY(bool playing READ playing NOTIFY frameChanged)
    Q_PROPERTY(double rate READ rate NOTIFY frameChanged)
    Q_PROPERTY(double recordedTime READ recordedTime NOTIFY frameChanged)
    Q_PROPERTY(double speedKmh READ speedKmh NOTIFY frameChanged)
    Q_PROPERTY(double acceleration READ acceleration NOTIFY frameChanged)
    Q_PROPERTY(double yawRateDeg READ yawRateDeg NOTIFY frameChanged)
    Q_PROPERTY(double latitude READ latitude NOTIFY frameChanged)
    Q_PROPERTY(double longitude READ longitude NOTIFY frameChanged)
    Q_PROPERTY(double positionAccuracy READ positionAccuracy NOTIFY frameChanged)
    Q_PROPERTY(int objectCount READ objectCount NOTIFY frameChanged)
    Q_PROPERTY(double nearestAhead READ nearestAhead NOTIFY frameChanged)
    Q_PROPERTY(QString cameraSource READ cameraSource NOTIFY frameChanged)
    Q_PROPERTY(QString lidarSource READ lidarSource NOTIFY frameChanged)
    Q_PROPERTY(QPointF position READ position NOTIFY frameChanged)
    Q_PROPERTY(QList<QPointF> trace READ trace NOTIFY changed)

public:
    explicit RecordedData(QObject* parent = nullptr);

    bool loaded() const { return playback_.loaded(); }
    QString error() const { return error_; }
    QStringList warnings() const { return warnings_; }
    QString name() const;
    QString source() const;
    QString license() const;
    QString summary() const;
    int frameCount() const;
    double duration() const;
    bool hasCamera() const;
    bool hasLidar() const;
    bool hasObjects() const;

    double time() const { return playback_.time(); }
    int frame() const { return playback_.frameIndex(); }
    bool playing() const { return playback_.playing(); }
    double rate() const { return playback_.rate(); }
    double recordedTime() const;
    double speedKmh() const;
    double acceleration() const;
    double yawRateDeg() const;
    double latitude() const;
    double longitude() const;
    double positionAccuracy() const;
    int objectCount() const;
    double nearestAhead() const;
    QString cameraSource() const;
    QString lidarSource() const;
    QPointF position() const;
    QList<QPointF> trace() const { return trace_; }

    // Accepts a plain path or a file:// URL.
    Q_INVOKABLE bool load(const QString& pathOrUrl);
    Q_INVOKABLE void unload();
    Q_INVOKABLE void play();
    Q_INVOKABLE void pause();
    Q_INVOKABLE void toggle();
    Q_INVOKABLE void seek(double seconds);
    Q_INVOKABLE void stepFrame(int delta);
    Q_INVOKABLE void setRate(double rate);
    // Full-duration series: channel 0 speed (km/h), 1 forward acceleration (m/s^2), 2 yaw rate (deg/s).
    Q_INVOKABLE void fillSeries(QAbstractSeries* series, int channel) const;
    Q_INVOKABLE void fillCursor(QAbstractSeries* series, double low, double high) const;

signals:
    void changed();       // a dataset was loaded or unloaded
    void frameChanged();  // time / frame / playback state changed

private:
    void tick();
    const driveforge::RecordedFrame* current() const { return playback_.frame(); }

    driveforge::RecordedPlayback playback_;
    QString error_;
    QStringList warnings_;
    QList<QPointF> trace_;
    QTimer timer_;
    QElapsedTimer clock_;
    qint64 lastNs_ = 0;
    int lastFrame_ = -1;
};
