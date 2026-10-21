#include <QtTest>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "Recorded.h"

using namespace driveforge;
namespace fs = std::filesystem;

// IMPORTANT: everything written by writeFixture() is a SYNTHETIC TEST FIXTURE in the KITTI file format, generated here
// only to exercise the parser with known values. It is never shipped, never shown in the app, and never presented as
// real data. The real-data test at the bottom uses the actual KITTI drive if it has been downloaded.

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarth = 6378137.0;

struct FixtureOptions {
    int frames = 6;
    bool camera = true;
    bool lidar = true;
    bool tracklets = true;
    bool boostRoot = true;
    std::string xmlOverride;  // replaces the tracklet XML entirely if non-empty
};

void writeFile(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream(path, std::ios::binary) << text;
}

std::string frameName(int i) {
    char b[32];
    std::snprintf(b, sizeof b, "%010d", i);
    return b;
}

std::string oxtsLine(int i) {
    const double lat0 = 49.0, lon0 = 8.0;
    const double metersPerDegLon = std::cos(lat0 * kPi / 180.0) * kPi * kEarth / 180.0;
    const double lon = lon0 + i * (1.0 / metersPerDegLon);  // 1 m east per frame
    char b[512];
    // lat lon alt roll pitch yaw vn ve vf vl vu ax ay az af al au wx wy wz wf wl wu pos_acc vel_acc navstat numsats posmode velmode orimode
    std::snprintf(b, sizeof b, "%.12f %.12f 100.0 0.01 0.02 0.0 0.0 10.0 10.0 0.5 0.0 0.0 0.0 9.8 1.0 0.2 9.8 0.0 0.0 0.1 0.0 0.0 0.1 0.01 0.02 4 10 5 5 5", lat0, lon);
    return b;
}

std::string trackletXml(bool boostRoot) {
    const std::string pose = [](double tx, int state) {
        char b[400];
        std::snprintf(b, sizeof b,
                      "<item class_id=\"3\" tracking_level=\"0\" version=\"2\"><tx>%g</tx><ty>0.5</ty><tz>-1.7</tz><rx>0</rx><ry>0</ry><rz>0.1</rz>"
                      "<state>%d</state><occlusion>0</occlusion><occlusion_kf>0</occlusion_kf><truncation>0</truncation><amt_occlusion>0</amt_occlusion>"
                      "<amt_occlusion_kf>-1</amt_occlusion_kf><amt_border_l>-1</amt_border_l><amt_border_r>-1</amt_border_r><amt_border_kf>-1</amt_border_kf></item>",
                      tx, state);
        return std::string(b);
    }(0, 0);
    (void)pose;
    auto p = [](double tx, int state) {
        char b[400];
        std::snprintf(b, sizeof b,
                      "<item><tx>%g</tx><ty>0.5</ty><tz>-1.7</tz><rx>0</rx><ry>0</ry><rz>0.1</rz><state>%d</state><occlusion>0</occlusion></item>", tx, state);
        return std::string(b);
    };
    std::string x = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>\n<!DOCTYPE boost_serialization>\n";
    if (boostRoot) x += "<boost_serialization signature=\"serialization::archive\" version=\"9\">\n";
    x += "<tracklets class_id=\"0\" tracking_level=\"0\" version=\"0\"><count>2</count><item_version>1</item_version>\n";
    // Track 0: a car, frames 1..4: pose states 2 (labelled), 1 (interpolated), 0 (unset: must be skipped), 2
    x += "<item class_id=\"1\" tracking_level=\"0\" version=\"1\"><objectType>Car</objectType><h>1.5</h><w>1.6</w><l>4</l><first_frame>1</first_frame>"
         "<poses class_id=\"2\" tracking_level=\"0\" version=\"0\"><count>4</count><item_version>1</item_version>" +
         p(20, 2) + p(19, 1) + p(18, 0) + p(17, 2) + "</poses><finished>1</finished></item>\n";
    // Track 1: a pedestrian far to the side, frame 0..1 (+ one pose beyond the last frame: must be ignored)
    x += "<item><objectType>Pedestrian</objectType><h>1.8</h><w>0.6</w><l>0.8</l><first_frame>4</first_frame>"
         "<poses><count>3</count><item_version>1</item_version>" + p(12, 2) + p(12, 2) + p(12, 2) + "</poses><finished>1</finished></item>\n";
    x += "</tracklets>\n";
    if (boostRoot) x += "</boost_serialization>\n";
    return x;
}

// Writes a KITTI-format drive folder and returns its path.
std::string writeFixture(const fs::path& base, const FixtureOptions& o = {}, const std::string& drive = "2011_09_26_drive_9999_sync") {
    const fs::path d = base / drive;
    std::string stamps;
    for (int i = 0; i < o.frames; ++i) {
        char b[64];
        std::snprintf(b, sizeof b, "2011-09-26 13:02:25.%09d\n", 100000000 * i);  // 0.1 s apart
        stamps += b;
        writeFile(d / "oxts/data" / (frameName(i) + ".txt"), oxtsLine(i) + "\n");
        if (o.camera) writeFile(d / "image_02/data" / (frameName(i) + ".png"), "x");
        if (o.lidar) {
            std::string bin(48, '\0');
            const float pts[12] = {10, 1, 0, 0.5f, 20, -2, 0.5f, 0.3f, 5, 5, 1, 0.9f};
            std::memcpy(bin.data(), pts, 48);
            writeFile(d / "velodyne_points/data" / (frameName(i) + ".bin"), bin);
        }
    }
    writeFile(d / "oxts/timestamps.txt", stamps);
    if (o.tracklets) writeFile(d / "tracklet_labels.xml", o.xmlOverride.empty() ? trackletXml(o.boostRoot) : o.xmlOverride);
    return d.string();
}

bool fails(const std::string& dir, const char* messagePart) {
    RecordedDataset ds;
    std::string err;
    if (loadKittiRaw(dir, &ds, &err)) return false;
    return err.find(messagePart) != std::string::npos;
}
}  // namespace

class RecordedTest : public QObject {
    Q_OBJECT
    QTemporaryDir tmp_;
    fs::path base(const char* name) { return fs::path(tmp_.path().toStdString()) / name; }

private slots:
    void initTestCase() { QVERIFY(tmp_.isValid()); }

    void loadsEgoMotionFromTheFiles() {
        const std::string dir = writeFixture(base("a"));
        RecordedDataset ds;
        std::string err;
        QVERIFY2(loadKittiRaw(dir, &ds, &err), err.c_str());
        QCOMPARE(ds.frames.size(), size_t(6));
        QCOMPARE(ds.name, std::string("2011_09_26_drive_9999_sync"));
        for (int i = 0; i < 6; ++i) {
            const RecordedFrame& f = ds.frames[size_t(i)];
            QVERIFY(std::abs(f.t - 0.1 * i) < 1e-9);
            QVERIFY2(std::abs(f.x - i) < 1e-3, qPrintable(QString("x[%1]=%2").arg(i).arg(f.x)));
            QVERIFY(std::abs(f.y) < 1e-6);
            QVERIFY(std::abs(f.speed - std::hypot(10.0, 0.5)) < 1e-9);
            QCOMPARE(f.aForward, 1.0);
            QCOMPARE(f.yawRate, 0.1);
            QCOMPARE(f.posAccuracy, 0.01);
        }
        QVERIFY(std::abs(ds.duration() - 0.5) < 1e-6);
        QVERIFY(std::abs(ds.pathLength() - 5.0) < 1e-2);
        QVERIFY(ds.hasCamera && ds.hasLidar && ds.hasObjects);
        QVERIFY(ds.warnings.empty());
        QVERIFY(ds.summary().find("6 frames") != std::string::npos);
    }

    void timestampsAcrossMidnight() {
        const std::string dir = writeFixture(base("mid"), {.frames = 2});
        writeFile(fs::path(dir) / "oxts/timestamps.txt", "2011-09-26 23:59:59.950000000\n2011-09-27 00:00:00.050000000\n");
        RecordedDataset ds;
        std::string err;
        QVERIFY2(loadKittiRaw(dir, &ds, &err), err.c_str());
        QVERIFY(std::abs(ds.frames[1].t - 0.1) < 1e-6);
    }

    void annotationsAreAttachedToTheRightFrames() {
        for (bool boost : {true, false}) {  // the real files have a <boost_serialization> root; also accept a bare <tracklets>
            FixtureOptions o;
            o.boostRoot = boost;
            RecordedDataset ds;
            std::string err;
            QVERIFY2(loadKittiRaw(writeFixture(base(boost ? "t1" : "t2"), o), &ds, &err), err.c_str());
            QCOMPARE(ds.trackCount, 2);
            QCOMPARE(ds.frames[0].objects.size(), size_t(0));
            // Car: frame 1 labelled, frame 2 interpolated, frame 3 unset (skipped), frame 4 labelled
            QCOMPARE(ds.frames[1].objects.size(), size_t(1));
            QCOMPARE(ds.frames[1].objects[0].type, std::string("Car"));
            QCOMPARE(ds.frames[1].objects[0].x, 20.0);
            QVERIFY(!ds.frames[1].objects[0].interpolated);
            QCOMPARE(ds.frames[1].objects[0].length, 4.0);
            QCOMPARE(ds.frames[1].objects[0].width, 1.6);
            QCOMPARE(ds.frames[2].objects.size(), size_t(1));
            QVERIFY(ds.frames[2].objects[0].interpolated);
            QCOMPARE(ds.frames[3].objects.size(), size_t(0));
            // Frame 4: the car again plus the pedestrian; frame 5: pedestrian only (its 3rd pose is beyond the last frame and ignored)
            QCOMPARE(ds.frames[4].objects.size(), size_t(2));
            QCOMPARE(ds.frames[5].objects.size(), size_t(1));
            QCOMPARE(ds.frames[5].objects[0].type, std::string("Pedestrian"));
        }
    }

    void nearestObjectAheadUsesTheCorridor() {
        RecordedFrame f;
        RecordedObject car;  car.x = 20; car.y = 0.5; car.length = 4; car.width = 1.6;
        RecordedObject side; side.x = 8;  side.y = 6;  side.length = 4; side.width = 1.6;     // other lane
        RecordedObject behind; behind.x = -5; behind.y = 0; behind.length = 4; behind.width = 1.6;
        f.objects = {side, behind};
        QCOMPARE(RecordedPlayback::nearestAhead(f), -1.0);
        f.objects.push_back(car);
        QVERIFY(std::abs(RecordedPlayback::nearestAhead(f) - 18.0) < 1e-9);
    }

    void missingImagesAreWarningsNotErrors() {
        const std::string dir = writeFixture(base("w"), {.camera = false});
        RecordedDataset ds;
        std::string err;
        QVERIFY2(loadKittiRaw(dir, &ds, &err), err.c_str());
        QVERIFY(!ds.hasCamera && ds.hasLidar);
        QVERIFY(!ds.warnings.empty());
        fs::remove(fs::path(dir) / "velodyne_points/data" / (frameName(2) + ".bin"));
        QVERIFY(loadKittiRaw(dir, &ds, &err));
        QVERIFY(ds.frames[2].lidarPath.empty() && !ds.frames[3].lidarPath.empty());
        bool partial = false;
        for (const auto& w : ds.warnings) partial = partial || w.find("1 frames have no lidar") != std::string::npos;
        QVERIFY(partial);
    }

    void findsTheDriveFolderBelowAParent() {
        const fs::path parent = base("nested");
        writeFixture(parent / "2011_09_26", {}, "2011_09_26_drive_0042_sync");
        RecordedDataset ds;
        std::string err;
        QVERIFY2(loadKittiRaw(parent.string(), &ds, &err), err.c_str());
        QCOMPARE(ds.name, std::string("2011_09_26_drive_0042_sync"));
    }

    // ---- bad input is rejected with a clear message, never crashes ----
    void rejectsMissingOrIncompleteData() {
        QVERIFY(fails((base("nope")).string(), "Not a folder"));
        fs::create_directories(base("empty"));
        QVERIFY(fails(base("empty").string(), "No KITTI drive found"));

        std::string dir = writeFixture(base("e1"));
        fs::remove(fs::path(dir) / "oxts/timestamps.txt");
        QVERIFY(fails(dir, "timestamps.txt is missing"));

        dir = writeFixture(base("e2"));
        fs::remove(fs::path(dir) / "oxts/data" / (frameName(3) + ".txt"));
        QVERIFY(fails(dir, "0000000003.txt is missing"));

        dir = writeFixture(base("e3"), {.frames = 1});
        QVERIFY(fails(dir, "fewer than 2 frames"));
    }

    void rejectsMalformedNumbersAndTimestamps() {
        std::string dir = writeFixture(base("m1"));
        writeFile(fs::path(dir) / "oxts/data" / (frameName(1) + ".txt"), "abc def\n");
        QVERIFY(fails(dir, "expected 30 numbers"));

        dir = writeFixture(base("m2"));
        std::string line = oxtsLine(1);
        line.replace(0, 4, "4x.0");
        writeFile(fs::path(dir) / "oxts/data" / (frameName(1) + ".txt"), line + "\n");
        QVERIFY(fails(dir, "invalid number"));

        dir = writeFixture(base("m3"));
        writeFile(fs::path(dir) / "oxts/data" / (frameName(1) + ".txt"), "95.0 8.0 " + oxtsLine(1).substr(oxtsLine(1).find("100.0")) + "\n");
        QVERIFY(fails(dir, "out of range"));

        dir = writeFixture(base("m4"));
        writeFile(fs::path(dir) / "oxts/timestamps.txt", "2011-09-26 13:02:25.0\nnot a time\n");
        QVERIFY(fails(dir, "cannot parse"));

        dir = writeFixture(base("m5"));
        writeFile(fs::path(dir) / "oxts/timestamps.txt", "2011-09-26 13:02:25.5\n2011-09-26 13:02:25.4\n");
        QVERIFY(fails(dir, "not increasing"));

        dir = writeFixture(base("m6"));
        std::string nan = oxtsLine(1);
        nan.replace(nan.find("100.0"), 5, "nan");
        writeFile(fs::path(dir) / "oxts/data" / (frameName(1) + ".txt"), nan + "\n");
        QVERIFY(fails(dir, "invalid number"));
    }

    void rejectsMalformedAnnotationFiles() {
        QVERIFY(fails(writeFixture(base("x1"), {.xmlOverride = "<tracklets><count>1</count><item><objectType>Car"}), "unexpected end"));
        QVERIFY(fails(writeFixture(base("x2"), {.xmlOverride = "<tracklets><item></tracklets>"}), "mismatched"));
        QVERIFY(fails(writeFixture(base("x3"), {.xmlOverride = "<tracklets><item><h>tall</h></item></tracklets>"}), "invalid number"));
        QVERIFY(fails(writeFixture(base("x4"), {.xmlOverride = "<something><else/></something>"}), "no <tracklets>"));
        QVERIFY(fails(writeFixture(base("x5"), {.xmlOverride = "<tracklets><item><poses><item><tx>1"}), "unexpected end"));
        QVERIFY(fails(writeFixture(base("x6"), {.xmlOverride = std::string("\x01\x02<<>><", 8)}), "tracklet_labels.xml"));
        // deeply nested garbage
        std::string deep = "<tracklets>";
        for (int i = 0; i < 40; ++i) deep += "<a>";
        QVERIFY(fails(writeFixture(base("x7"), {.xmlOverride = deep}), "nesting too deep"));
    }

    void annotationsAreOptional() {
        RecordedDataset ds;
        std::string err;
        QVERIFY2(loadKittiRaw(writeFixture(base("noxml"), {.tracklets = false}), &ds, &err), err.c_str());
        QVERIFY(!ds.hasObjects);
        QVERIFY(!ds.warnings.empty());
    }

    void readsLidarAndRejectsBadScans() {
        const std::string dir = writeFixture(base("l"));
        std::vector<LidarPoint> pts;
        std::string err;
        QVERIFY2(readVelodyne(dir + "/velodyne_points/data/0000000000.bin", &pts, &err), err.c_str());
        QCOMPARE(pts.size(), size_t(3));
        QCOMPARE(pts[1].x, 20.0f);
        QCOMPARE(pts[1].y, -2.0f);
        QCOMPARE(pts[2].intensity, 0.9f);

        writeFile(fs::path(dir) / "bad.bin", std::string(47, 'x'));
        QVERIFY(!readVelodyne(dir + "/bad.bin", &pts, &err));
        QVERIFY(err.find("truncated") != std::string::npos);
        writeFile(fs::path(dir) / "empty.bin", "");
        QVERIFY(!readVelodyne(dir + "/empty.bin", &pts, &err));
        QVERIFY(!readVelodyne(dir + "/missing.bin", &pts, &err));
        writeFile(fs::path(dir) / "huge.bin", std::string(17 * 1024 * 1024, '\0'));
        QVERIFY(!readVelodyne(dir + "/huge.bin", &pts, &err));
        QVERIFY(err.find("unreasonably large") != std::string::npos);
    }

    // ---- playback ----
    void playbackFollowsRecordedTimestamps() {
        auto ds = std::make_shared<RecordedDataset>();
        std::string err;
        QVERIFY2(loadKittiRaw(writeFixture(base("p")), ds.get(), &err), err.c_str());
        RecordedPlayback pb(ds);
        QVERIFY(pb.loaded());
        pb.advance(1.0);  // paused: nothing moves
        QCOMPARE(pb.time(), 0.0);
        QCOMPARE(pb.frameIndex(), 0);

        pb.play();
        pb.advance(0.25);
        QCOMPARE(pb.frameIndex(), 2);  // frame 2 is held until frame 3's timestamp (0.3 s); no interpolation
        QVERIFY(std::abs(pb.frame()->t - 0.2) < 1e-9);
        pb.advance(0.05);
        QCOMPARE(pb.frameIndex(), 3);

        pb.setRate(2.0);
        pb.advance(0.05);  // 0.1 s of recorded time
        QCOMPARE(pb.frameIndex(), 4);

        pb.advance(10.0);  // runs past the end: clamps and stops
        QVERIFY(!pb.playing());
        QCOMPARE(pb.frameIndex(), 5);
        QCOMPARE(pb.time(), ds->duration());
        pb.play();  // restarts from the beginning
        QCOMPARE(pb.time(), 0.0);
        QVERIFY(pb.playing());
    }

    void seekingAndSteppingStayInRange() {
        auto ds = std::make_shared<RecordedDataset>();
        std::string err;
        QVERIFY(loadKittiRaw(writeFixture(base("s")), ds.get(), &err));
        RecordedPlayback pb(ds);
        pb.seek(-3.0);
        QCOMPARE(pb.time(), 0.0);
        pb.seek(99.0);
        QCOMPARE(pb.frameIndex(), 5);
        pb.seekFrame(2);
        QVERIFY(std::abs(pb.time() - 0.2) < 1e-9);
        pb.stepFrame(-10);
        QCOMPARE(pb.frameIndex(), 0);
        pb.stepFrame(1);
        QCOMPARE(pb.frameIndex(), 1);
        pb.stepFrame(100);
        QCOMPARE(pb.frameIndex(), 5);
        pb.setRate(100.0);
        QCOMPARE(pb.rate(), 8.0);
        pb.setRate(0.0);
        QCOMPARE(pb.rate(), 0.1);
    }

    void playbackWithoutDataIsHarmless() {
        RecordedPlayback pb;
        QVERIFY(!pb.loaded());
        pb.play();
        pb.advance(1.0);
        pb.seek(1.0);
        pb.stepFrame(1);
        QVERIFY(!pb.playing());
        QVERIFY(pb.frame() == nullptr);
        QCOMPARE(pb.frameIndex(), 0);
    }

    // ---- the real KITTI drive, if it has been downloaded (see README) ----
    void realKittiDrive() {
        const char* env = std::getenv("DRIVEFORGE_KITTI_DIR");
        const std::string dir = env ? std::string(env) : std::string(SOURCE_DIR) + "/data/kitti";
        if (!fs::is_directory(dir)) QSKIP("real KITTI data not downloaded (see README: Recorded data)");
        RecordedDataset ds;
        std::string err;
        QVERIFY2(loadKittiRaw(dir, &ds, &err), err.c_str());
        qInfo() << QString::fromStdString(ds.summary());

        QVERIFY(ds.frames.size() >= 50);
        QVERIFY(ds.duration() > 5.0 && ds.duration() < 600.0);
        // The recording runs at ~10 Hz.
        const double dt = ds.duration() / double(ds.frames.size() - 1);
        QVERIFY2(dt > 0.08 && dt < 0.12, qPrintable(QString("mean frame interval %1 s").arg(dt)));
        // A car on a road: plausible speeds, small yaw rates, GPS accuracy of a few metres at most.
        double vmax = 0;
        for (const auto& f : ds.frames) {
            vmax = std::max(vmax, f.speed);
            QVERIFY(f.speed >= 0.0 && f.speed < 60.0);
            QVERIFY(std::abs(f.yawRate) < 1.0);
            QVERIFY(f.posAccuracy >= 0.0 && f.posAccuracy < 50.0);
        }
        QVERIFY(vmax > 1.0);
        // The distance integrated from the recorded speed must agree with the distance between recorded GPS positions.
        double integrated = 0.0;
        for (size_t i = 1; i < ds.frames.size(); ++i) integrated += 0.5 * (ds.frames[i].speed + ds.frames[i - 1].speed) * (ds.frames[i].t - ds.frames[i - 1].t);
        QVERIFY2(std::abs(integrated - ds.pathLength()) < 0.1 * ds.pathLength() + 1.0,
                 qPrintable(QString("speed integral %1 m vs GPS path %2 m").arg(integrated).arg(ds.pathLength())));
        QVERIFY(ds.hasCamera);
        QVERIFY(ds.hasLidar);
        std::vector<LidarPoint> pts;
        QVERIFY2(readVelodyne(ds.frames[0].lidarPath, &pts, &err), err.c_str());
        QVERIFY(pts.size() > 50000);
    }
};

QTEST_APPLESS_MAIN(RecordedTest)
#include "tst_recorded.moc"
