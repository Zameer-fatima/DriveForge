#include "RecordedData.h"

#include <QDir>
#include <QFileInfo>
#include <QImage>
#include <QPainter>
#include <QPolygonF>
#include <QUrl>
#include <QtCharts/QXYSeries>
#include <cmath>

using namespace driveforge;

namespace {
std::mutex g_mutex;
std::shared_ptr<const RecordedDataset> g_data;
int g_serial = 0;

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

// Top-down lidar view: x forward (up) from -10 to +60 m, y left (left) +/-30 m, 8 px per metre.
constexpr double kPxPerM = 8.0;
constexpr double kForwardMax = 60.0, kBackMax = 10.0, kHalfWidth = 30.0;

QPointF toPixel(double x, double y) { return {(kHalfWidth - y) * kPxPerM, (kForwardMax - x) * kPxPerM}; }

QColor typeColor(const std::string& type) {
    if (type == "Car") return QColor("#3ddc84");
    if (type == "Van" || type == "Truck" || type == "Tram") return QColor("#27c9ff");
    if (type == "Pedestrian" || type == "Person_sitting") return QColor("#ff5d5d");
    if (type == "Cyclist") return QColor("#ffd21f");
    return QColor("#d8d8d8");
}

void label(QPainter& p, const QRect& area, const QString& text) {
    QFont f = p.font();
    f.setPixelSize(std::max(11, area.height() / 22));
    f.setBold(true);
    p.setFont(f);
    const QRect box(area.left() + 4, area.top() + 4, p.fontMetrics().horizontalAdvance(text) + 12, p.fontMetrics().height() + 6);
    p.fillRect(box, QColor(180, 110, 0, 220));
    p.setPen(Qt::white);
    p.drawText(box, Qt::AlignCenter, text);
}

QImage renderLidar(const RecordedFrame& frame) {
    const int w = static_cast<int>(2 * kHalfWidth * kPxPerM), h = static_cast<int>((kForwardMax + kBackMax) * kPxPerM);
    QImage img(w, h, QImage::Format_RGB32);
    img.fill(QColor("#0b0f14"));
    QPainter p(&img);
    p.setRenderHint(QPainter::Antialiasing, true);

    // Range rings
    p.setPen(QPen(QColor(255, 255, 255, 40), 1));
    for (int r = 10; r <= 60; r += 10) {
        const QPointF c = toPixel(0, 0);
        p.drawEllipse(c, r * kPxPerM, r * kPxPerM);
    }

    // Points, coloured by height
    std::vector<LidarPoint> pts;
    std::string err;
    if (!frame.lidarPath.empty() && readVelodyne(frame.lidarPath, &pts, &err)) {
        for (const LidarPoint& q : pts) {
            if (q.x < -kBackMax || q.x > kForwardMax || std::abs(q.y) > kHalfWidth || q.z < -2.8f || q.z > 3.0f) continue;
            const double t = std::min(std::max((q.z + 2.5) / 4.0, 0.0), 1.0);
            const QColor c = QColor::fromHsvF(0.62 - 0.55 * t, 0.85, 0.55 + 0.45 * t);
            const QPointF px = toPixel(q.x, q.y);
            const int ix = static_cast<int>(px.x()), iy = static_cast<int>(px.y());
            if (ix < 0 || iy < 0 || ix + 1 >= w || iy + 1 >= h) continue;
            img.setPixel(ix, iy, c.rgb());
            img.setPixel(ix + 1, iy, c.rgb());
        }
    } else {
        p.setPen(QColor("#ffb454"));
        p.drawText(QRect(0, 0, w, h), Qt::AlignCenter, "no lidar scan for this frame");
    }

    // Annotation boxes (human labels)
    for (const RecordedObject& o : frame.objects) {
        const double cy = std::cos(o.yaw), sy = std::sin(o.yaw);
        const double hl = o.length / 2.0, hw = o.width / 2.0;
        QPolygonF poly;
        for (const auto& corner : {std::pair{hl, hw}, std::pair{hl, -hw}, std::pair{-hl, -hw}, std::pair{-hl, hw}})
            poly << toPixel(o.x + corner.first * cy - corner.second * sy, o.y + corner.first * sy + corner.second * cy);
        QPen pen(typeColor(o.type), 2);
        if (o.interpolated) pen.setStyle(Qt::DashLine);
        p.setPen(pen);
        p.setBrush(Qt::NoBrush);
        p.drawPolygon(poly);
        p.drawText(poly.boundingRect().topRight() + QPointF(3, 10), QString::fromStdString(o.type));
    }

    // Sensor position (the lidar sits on the car roof)
    p.setPen(QPen(Qt::white, 1));
    p.setBrush(QColor("#4cc3ff"));
    const QPointF o = toPixel(0, 0);
    p.drawPolygon(QPolygonF({o + QPointF(0, -9), o + QPointF(-6, 7), o + QPointF(6, 7)}));
    label(p, QRect(0, 0, w, h), "RECORDED lidar (KITTI, real)");
    return img;
}
}  // namespace

// ---- store / provider ---------------------------------------------------------------------------------------------

std::shared_ptr<const RecordedDataset> RecordedStore::get() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_data;
}
int RecordedStore::serial() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_serial;
}
void RecordedStore::set(std::shared_ptr<const RecordedDataset> data) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_data = std::move(data);
    ++g_serial;
}

QImage RecordedImageProvider::requestImage(const QString& id, QSize* size, const QSize&) {
    const QStringList parts = id.split('/');
    QImage out;
    if (parts.size() == 3) {
        const auto data = RecordedStore::get();
        bool okSerial = false, okFrame = false;
        const int serial = parts[1].toInt(&okSerial);
        const int index = parts[2].toInt(&okFrame);
        if (data && okSerial && okFrame && serial == RecordedStore::serial() && index >= 0 && index < static_cast<int>(data->frames.size())) {
            const RecordedFrame& f = data->frames[static_cast<std::size_t>(index)];
            if (parts[0] == "camera") {
                if (!f.cameraPath.empty() && out.load(QString::fromStdString(f.cameraPath))) {
                    out = out.convertToFormat(QImage::Format_RGB32);
                    QPainter p(&out);
                    p.setRenderHint(QPainter::TextAntialiasing, true);
                    label(p, out.rect(), "RECORDED camera (KITTI, real)");
                }
            } else if (parts[0] == "lidar") {
                out = renderLidar(f);
            }
        }
    }
    if (out.isNull()) {
        out = QImage(320, 120, QImage::Format_RGB32);
        out.fill(QColor("#1a1f26"));
        QPainter p(&out);
        p.setPen(QColor("#8fa3b8"));
        p.drawText(out.rect(), Qt::AlignCenter, "no recorded image for this frame");
    }
    if (size) *size = out.size();
    return out;
}

// ---- RecordedData -------------------------------------------------------------------------------------------------

RecordedData::RecordedData(QObject* parent) : QObject(parent) {
    timer_.setTimerType(Qt::PreciseTimer);
    timer_.setInterval(16);
    connect(&timer_, &QTimer::timeout, this, &RecordedData::tick);
    clock_.start();
    lastNs_ = clock_.nsecsElapsed();
}

QString RecordedData::name() const { return loaded() ? QString::fromStdString(playback_.dataset()->name) : QString(); }
QString RecordedData::source() const { return loaded() ? QString::fromStdString(playback_.dataset()->source) : QString(); }
QString RecordedData::license() const { return loaded() ? QString::fromStdString(playback_.dataset()->license) : QString(); }
QString RecordedData::summary() const { return loaded() ? QString::fromStdString(playback_.dataset()->summary()) : QString(); }
int RecordedData::frameCount() const { return loaded() ? static_cast<int>(playback_.dataset()->frames.size()) : 0; }
double RecordedData::duration() const { return loaded() ? playback_.dataset()->duration() : 0.0; }
bool RecordedData::hasCamera() const { return loaded() && playback_.dataset()->hasCamera; }
bool RecordedData::hasLidar() const { return loaded() && playback_.dataset()->hasLidar; }
bool RecordedData::hasObjects() const { return loaded() && playback_.dataset()->hasObjects; }
double RecordedData::recordedTime() const { return current() ? current()->t : 0.0; }
double RecordedData::speedKmh() const { return current() ? current()->speed * 3.6 : 0.0; }
double RecordedData::acceleration() const { return current() ? current()->aForward : 0.0; }
double RecordedData::yawRateDeg() const { return current() ? current()->yawRate * kRadToDeg : 0.0; }
double RecordedData::latitude() const { return current() ? current()->latitude : 0.0; }
double RecordedData::longitude() const { return current() ? current()->longitude : 0.0; }
double RecordedData::positionAccuracy() const { return current() ? current()->posAccuracy : 0.0; }
int RecordedData::objectCount() const { return current() ? static_cast<int>(current()->objects.size()) : 0; }
double RecordedData::nearestAhead() const { return current() ? RecordedPlayback::nearestAhead(*current()) : -1.0; }
QPointF RecordedData::position() const { return current() ? QPointF(current()->x, current()->y) : QPointF(); }

QString RecordedData::cameraSource() const {
    return loaded() ? QString("image://recorded/camera/%1/%2").arg(RecordedStore::serial()).arg(frame()) : QString();
}
QString RecordedData::lidarSource() const {
    return loaded() ? QString("image://recorded/lidar/%1/%2").arg(RecordedStore::serial()).arg(frame()) : QString();
}

bool RecordedData::load(const QString& pathOrUrl) {
    QString path = pathOrUrl.trimmed();
    if (path.startsWith("file:")) path = QUrl(path).toLocalFile();
    if (path.startsWith("~/")) path = QDir::homePath() + path.mid(1);

    auto data = std::make_shared<RecordedDataset>();
    std::string err;
    if (!loadKittiRaw(path.toStdString(), data.get(), &err)) {
        error_ = QString::fromStdString(err);
        warnings_.clear();
        emit changed();
        return false;
    }
    error_.clear();
    warnings_.clear();
    for (const std::string& w : data->warnings) warnings_ << QString::fromStdString(w);
    trace_.clear();
    for (const RecordedFrame& f : data->frames) trace_.append(QPointF(f.x, f.y));
    RecordedStore::set(data);
    playback_.setDataset(data);
    lastFrame_ = -1;
    timer_.stop();
    emit changed();
    emit frameChanged();
    return true;
}

void RecordedData::unload() {
    timer_.stop();
    playback_.setDataset(nullptr);
    RecordedStore::set(nullptr);
    trace_.clear();
    warnings_.clear();
    error_.clear();
    emit changed();
    emit frameChanged();
}

void RecordedData::play() {
    playback_.play();
    lastNs_ = clock_.nsecsElapsed();
    if (playback_.playing()) timer_.start();
    emit frameChanged();
}

void RecordedData::pause() {
    playback_.pause();
    timer_.stop();
    emit frameChanged();
}

void RecordedData::toggle() { playback_.playing() ? pause() : play(); }
void RecordedData::seek(double seconds) { playback_.seek(seconds); emit frameChanged(); }
void RecordedData::stepFrame(int delta) { playback_.pause(); timer_.stop(); playback_.stepFrame(delta); emit frameChanged(); }
void RecordedData::setRate(double rate) { playback_.setRate(rate); emit frameChanged(); }

void RecordedData::tick() {
    const qint64 now = clock_.nsecsElapsed();
    const double dt = std::min(0.25, static_cast<double>(now - lastNs_) * 1e-9);
    lastNs_ = now;
    playback_.advance(dt);
    if (!playback_.playing()) timer_.stop();
    emit frameChanged();
}

void RecordedData::fillSeries(QAbstractSeries* series, int channel) const {
    auto* xy = qobject_cast<QXYSeries*>(series);
    if (!xy || !loaded()) return;
    QList<QPointF> pts;
    pts.reserve(frameCount());
    for (const RecordedFrame& f : playback_.dataset()->frames) {
        const double v = channel == 0 ? f.speed * 3.6 : channel == 1 ? f.aForward : f.yawRate * kRadToDeg;
        pts.append(QPointF(f.t, v));
    }
    xy->replace(pts);
}

void RecordedData::fillCursor(QAbstractSeries* series, double low, double high) const {
    auto* xy = qobject_cast<QXYSeries*>(series);
    if (!xy) return;
    const double t = current() ? current()->t : 0.0;
    xy->replace(QList<QPointF>{QPointF(t, low), QPointF(t, high)});
}
