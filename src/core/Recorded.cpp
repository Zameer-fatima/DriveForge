#include "Recorded.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace driveforge {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::uintmax_t kMaxTextFile = 50u * 1024u * 1024u;  // tracklet XML etc.
constexpr std::uintmax_t kMaxLidarFile = 16u * 1024u * 1024u;
constexpr int kMaxFrames = 100000;
constexpr int kMaxTracklets = 20000;
constexpr int kMaxPoses = 2000000;

std::string frameName(int i) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%010d", i);
    return buf;
}

bool fail(std::string* error, const std::string& message) {
    if (error) *error = message;
    return false;
}

bool readText(const fs::path& path, std::string* text, std::string* error) {
    std::error_code ec;
    const auto size = fs::file_size(path, ec);
    if (ec) return fail(error, "Cannot read " + path.string());
    if (size > kMaxTextFile) return fail(error, path.filename().string() + " is unreasonably large");
    std::ifstream in(path, std::ios::binary);
    if (!in) return fail(error, "Cannot open " + path.string());
    std::ostringstream ss;
    ss << in.rdbuf();
    *text = ss.str();
    return true;
}

// Strict number parsing: the whole token must be a finite number.
bool parseDouble(const std::string& token, double* value) {
    if (token.empty()) return false;
    char* end = nullptr;
    const double v = std::strtod(token.c_str(), &end);
    if (end != token.c_str() + token.size() || !std::isfinite(v)) return false;
    *value = v;
    return true;
}

std::vector<std::string> splitWhitespace(const std::string& line) {
    std::vector<std::string> out;
    std::istringstream ss(line);
    std::string tok;
    while (ss >> tok) out.push_back(tok);
    return out;
}

// days since 1970-01-01 for a civil date
long daysFromCivil(long y, unsigned m, unsigned d) {
    y -= m <= 2;
    const long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<long>(doe) - 719468;
}

// "2011-09-26 13:02:25.964389445" -> whole days since 1970-01-01 and seconds into that day (kept apart so that
// differences between timestamps stay precise; a single epoch-seconds double only resolves ~0.2 microseconds).
struct Stamp {
    long days = 0;
    double seconds = 0.0;
    double minus(const Stamp& o) const { return static_cast<double>(days - o.days) * 86400.0 + (seconds - o.seconds); }
};

bool parseTimestamp(const std::string& line, Stamp* out) {
    int Y, M, D, h, m;
    double s;
    if (std::sscanf(line.c_str(), "%d-%d-%d %d:%d:%lf", &Y, &M, &D, &h, &m, &s) != 6) return false;
    if (M < 1 || M > 12 || D < 1 || D > 31 || h < 0 || h > 23 || m < 0 || m > 59 || s < 0.0 || s >= 61.0) return false;
    out->days = daysFromCivil(Y, static_cast<unsigned>(M), static_cast<unsigned>(D));
    out->seconds = h * 3600.0 + m * 60.0 + s;
    return true;
}

// ---- minimal XML scanner for the KITTI tracklet file --------------------------------------------------------------

struct Tracklet {
    std::string type;
    double h = 0, w = 0, l = 0;
    int firstFrame = 0;
    struct Pose { double tx, ty, tz, rx, ry, rz; int state; };
    std::vector<Pose> poses;
};

bool parseTracklets(const std::string& xml, std::vector<Tracklet>* out, std::string* error) {
    std::vector<std::string> path;
    Tracklet current;
    Tracklet::Pose pose{};
    bool inTracklet = false, inPose = false, sawRoot = false;
    long poseCount = 0;

    // Path relative to the <tracklets> element (the real files wrap it in a <boost_serialization> root).
    auto joined = [&] {
        std::string s;
        bool on = false;
        for (const auto& p : path) {
            if (p == "tracklets") on = true;
            if (on) s += (s.empty() ? "" : "/") + p;
        }
        return s;
    };
    auto badNumber = [&](const std::string& tag, const std::string& text) {
        return fail(error, "tracklet_labels.xml: invalid number '" + text + "' in <" + tag + ">");
    };

    std::size_t i = 0;
    while (i < xml.size()) {
        const std::size_t lt = xml.find('<', i);
        if (lt == std::string::npos) break;
        // Text before this tag belongs to the innermost open element.
        std::string text = xml.substr(i, lt - i);
        const std::size_t gt = xml.find('>', lt);
        if (gt == std::string::npos) return fail(error, "tracklet_labels.xml: unterminated tag");
        std::string tag = xml.substr(lt + 1, gt - lt - 1);
        i = gt + 1;

        if (!tag.empty() && (tag[0] == '?' || tag[0] == '!')) continue;  // declaration, doctype, comment
        const bool closing = !tag.empty() && tag[0] == '/';
        const bool selfClosing = !tag.empty() && tag.back() == '/';
        if (closing) tag.erase(0, 1);
        if (selfClosing) tag.pop_back();
        const std::size_t sp = tag.find_first_of(" \t\r\n");
        const std::string name = sp == std::string::npos ? tag : tag.substr(0, sp);
        if (name.empty()) return fail(error, "tracklet_labels.xml: empty tag name");

        if (!closing) {
            path.push_back(name);
            const std::string p = joined();
            if (p == "tracklets") sawRoot = true;
            if (p == "tracklets/item") { current = Tracklet{}; inTracklet = true; }
            if (p == "tracklets/item/poses/item") { pose = Tracklet::Pose{}; pose.state = 0; inPose = true; ++poseCount; if (poseCount > kMaxPoses) return fail(error, "tracklet_labels.xml: too many poses"); }
            if (path.size() > 8) return fail(error, "tracklet_labels.xml: nesting too deep");
            if (selfClosing) path.pop_back();
            continue;
        }

        // Closing tag: interpret the text of the element being closed.
        if (path.empty() || path.back() != name) return fail(error, "tracklet_labels.xml: mismatched closing tag </" + name + ">");
        const std::string p = joined();
        while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) text.pop_back();
        std::size_t b = 0;
        while (b < text.size() && std::isspace(static_cast<unsigned char>(text[b]))) ++b;
        text = text.substr(b);

        double v = 0.0;
        auto num = [&](double* dst) { if (!parseDouble(text, &v)) return false; *dst = v; return true; };
        if (inPose && p.rfind("tracklets/item/poses/item/", 0) == 0) {
            const std::string f = p.substr(26);
            bool ok = true;
            if (f == "tx") ok = num(&pose.tx);
            else if (f == "ty") ok = num(&pose.ty);
            else if (f == "tz") ok = num(&pose.tz);
            else if (f == "rx") ok = num(&pose.rx);
            else if (f == "ry") ok = num(&pose.ry);
            else if (f == "rz") ok = num(&pose.rz);
            else if (f == "state") { double s; ok = num(&s); pose.state = static_cast<int>(s); }
            if (!ok) return badNumber(f, text);
        } else if (inTracklet && p == "tracklets/item/objectType") {
            current.type = text;
        } else if (inTracklet && p == "tracklets/item/h") { if (!num(&current.h)) return badNumber("h", text); }
        else if (inTracklet && p == "tracklets/item/w") { if (!num(&current.w)) return badNumber("w", text); }
        else if (inTracklet && p == "tracklets/item/l") { if (!num(&current.l)) return badNumber("l", text); }
        else if (inTracklet && p == "tracklets/item/first_frame") { double f; if (!num(&f)) return badNumber("first_frame", text); current.firstFrame = static_cast<int>(f); }
        else if (p == "tracklets/item/poses/item") { current.poses.push_back(pose); inPose = false; }
        else if (p == "tracklets/item") {
            out->push_back(current);
            inTracklet = false;
            if (static_cast<int>(out->size()) > kMaxTracklets) return fail(error, "tracklet_labels.xml: too many tracklets");
        }
        path.pop_back();
    }
    if (!path.empty()) return fail(error, "tracklet_labels.xml: unexpected end of file (unclosed <" + path.back() + ">)");
    if (!sawRoot) return fail(error, "tracklet_labels.xml: no <tracklets> element");
    return true;
}

// Finds the folder that contains oxts/ (the drive folder), searching up to three levels down.
bool findDriveDir(const fs::path& root, fs::path* drive, int depth = 0) {
    std::error_code ec;
    if (fs::is_directory(root / "oxts", ec)) { *drive = root; return true; }
    if (depth >= 3) return false;
    std::vector<fs::path> dirs;
    for (const auto& e : fs::directory_iterator(root, ec))
        if (e.is_directory(ec)) dirs.push_back(e.path());
    std::sort(dirs.begin(), dirs.end());
    for (const auto& d : dirs)
        if (findDriveDir(d, drive, depth + 1)) return true;
    return false;
}

}  // namespace

double RecordedDataset::pathLength() const {
    double len = 0.0;
    for (std::size_t i = 1; i < frames.size(); ++i) len += std::hypot(frames[i].x - frames[i - 1].x, frames[i].y - frames[i - 1].y);
    return len;
}

std::string RecordedDataset::summary() const {
    char buf[512];
    double vmax = 0.0, vmean = 0.0;
    for (const auto& f : frames) { vmax = std::max(vmax, f.speed); vmean += f.speed; }
    if (!frames.empty()) vmean /= static_cast<double>(frames.size());
    std::snprintf(buf, sizeof buf, "%s: %zu frames, %.1f s, %.0f m driven, speed mean %.1f / max %.1f m/s; camera %s, lidar %s, annotations %s (%d tracks)",
                  name.c_str(), frames.size(), duration(), pathLength(), vmean, vmax, hasCamera ? "yes" : "no", hasLidar ? "yes" : "no",
                  hasObjects ? "yes" : "no", trackCount);
    return buf;
}

bool loadKittiRaw(const std::string& dirArg, RecordedDataset* out, std::string* error) {
    std::error_code ec;
    fs::path root(dirArg);
    if (!fs::is_directory(root, ec)) return fail(error, "Not a folder: " + dirArg);
    fs::path drive;
    if (!findDriveDir(root, &drive)) return fail(error, "No KITTI drive found in " + dirArg + " (expected a folder containing oxts/data and oxts/timestamps.txt)");

    // Timestamps
    std::string text;
    if (!fs::is_regular_file(drive / "oxts" / "timestamps.txt", ec)) return fail(error, "oxts/timestamps.txt is missing");
    if (!readText(drive / "oxts" / "timestamps.txt", &text, error)) return false;
    std::vector<Stamp> stamps;
    {
        std::istringstream ss(text);
        std::string line;
        int lineNo = 0;
        while (std::getline(ss, line)) {
            ++lineNo;
            if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;
            Stamp s;
            if (!parseTimestamp(line, &s)) return fail(error, "oxts/timestamps.txt line " + std::to_string(lineNo) + ": cannot parse '" + line + "'");
            if (!stamps.empty() && s.minus(stamps.back()) <= 0.0) return fail(error, "oxts/timestamps.txt line " + std::to_string(lineNo) + ": timestamps are not increasing");
            stamps.push_back(s);
            if (static_cast<int>(stamps.size()) > kMaxFrames) return fail(error, "too many frames");
        }
    }
    if (stamps.size() < 2) return fail(error, "oxts/timestamps.txt has fewer than 2 frames");

    RecordedDataset ds;
    ds.directory = drive.string();
    ds.name = drive.filename().string();
    ds.frames.resize(stamps.size());

    // OXTS records
    double scale = 1.0;
    double mx0 = 0.0, my0 = 0.0;
    const double earth = 6378137.0;
    for (std::size_t i = 0; i < stamps.size(); ++i) {
        const fs::path file = drive / "oxts" / "data" / (frameName(static_cast<int>(i)) + ".txt");
        if (!fs::is_regular_file(file, ec)) return fail(error, "oxts/data/" + frameName(static_cast<int>(i)) + ".txt is missing (timestamps list " + std::to_string(stamps.size()) + " frames)");
        if (!readText(file, &text, error)) return false;
        const std::vector<std::string> tok = splitWhitespace(text);
        if (tok.size() < 25) return fail(error, file.filename().string() + ": expected 30 numbers, found " + std::to_string(tok.size()));
        double v[30] = {0};
        const std::size_t n = std::min<std::size_t>(tok.size(), 30);
        for (std::size_t k = 0; k < n; ++k)
            if (!parseDouble(tok[k], &v[k])) return fail(error, file.filename().string() + ": invalid number '" + tok[k] + "'");
        if (std::abs(v[0]) > 90.0 || std::abs(v[1]) > 180.0) return fail(error, file.filename().string() + ": latitude/longitude out of range");

        RecordedFrame& f = ds.frames[i];
        f.index = static_cast<int>(i);
        f.t = stamps[i].minus(stamps[0]);
        f.latitude = v[0]; f.longitude = v[1]; f.altitude = v[2];
        f.roll = v[3]; f.pitch = v[4]; f.yaw = v[5];
        f.vForward = v[8]; f.vLeft = v[9];
        f.speed = std::hypot(v[8], v[9]);
        f.aForward = v[14]; f.aLeft = v[15];
        f.yawRate = v[19];
        f.posAccuracy = v[23];
        if (i == 0) scale = std::cos(f.latitude * kPi / 180.0);
        // Mercator projection around the first fix (as in the KITTI devkit).
        const double mx = scale * f.longitude * kPi * earth / 180.0;
        const double my = scale * earth * std::log(std::tan((90.0 + f.latitude) * kPi / 360.0));
        if (i == 0) { mx0 = mx; my0 = my; }
        f.x = mx - mx0;
        f.y = my - my0;

        const fs::path cam = drive / "image_02" / "data" / (frameName(static_cast<int>(i)) + ".png");
        const fs::path lid = drive / "velodyne_points" / "data" / (frameName(static_cast<int>(i)) + ".bin");
        if (fs::is_regular_file(cam, ec)) { f.cameraPath = cam.string(); ds.hasCamera = true; }
        if (fs::is_regular_file(lid, ec)) { f.lidarPath = lid.string(); ds.hasLidar = true; }
    }
    int noCam = 0, noLid = 0;
    for (const auto& f : ds.frames) { noCam += f.cameraPath.empty(); noLid += f.lidarPath.empty(); }
    if (ds.hasCamera && noCam) ds.warnings.push_back(std::to_string(noCam) + " frames have no camera image");
    if (ds.hasLidar && noLid) ds.warnings.push_back(std::to_string(noLid) + " frames have no lidar scan");
    if (!ds.hasCamera) ds.warnings.push_back("no camera images (image_02/data) found");
    if (!ds.hasLidar) ds.warnings.push_back("no lidar scans (velodyne_points/data) found");

    // Annotations (optional)
    const fs::path xmlPath = drive / "tracklet_labels.xml";
    if (fs::is_regular_file(xmlPath, ec)) {
        std::vector<Tracklet> tracks;
        if (!readText(xmlPath, &text, error)) return false;
        if (!parseTracklets(text, &tracks, error)) return false;
        int id = 0;
        for (const Tracklet& tr : tracks) {
            for (std::size_t k = 0; k < tr.poses.size(); ++k) {
                const int fi = tr.firstFrame + static_cast<int>(k);
                if (fi < 0 || fi >= static_cast<int>(ds.frames.size())) continue;
                const auto& p = tr.poses[k];
                if (p.state != 1 && p.state != 2) continue;  // 0 = unset
                RecordedObject o;
                o.trackId = id;
                o.type = tr.type;
                o.x = p.tx; o.y = p.ty; o.z = p.tz; o.yaw = p.rz;
                o.length = tr.l; o.width = tr.w; o.height = tr.h;
                o.interpolated = p.state == 1;
                ds.frames[static_cast<std::size_t>(fi)].objects.push_back(o);
            }
            ++id;
        }
        ds.trackCount = id;
        ds.hasObjects = id > 0;
    } else {
        ds.warnings.push_back("no tracklet_labels.xml: object annotations are not available");
    }

    *out = std::move(ds);
    return true;
}

bool readVelodyne(const std::string& path, std::vector<LidarPoint>* points, std::string* error) {
    std::error_code ec;
    const auto size = fs::file_size(path, ec);
    if (ec) return fail(error, "Cannot read " + path);
    if (size == 0) return fail(error, "Lidar scan is empty: " + path);
    if (size > kMaxLidarFile) return fail(error, "Lidar scan is unreasonably large: " + path);
    if (size % sizeof(LidarPoint) != 0) return fail(error, "Lidar scan is truncated or not float32 x/y/z/intensity: " + path);
    std::ifstream in(path, std::ios::binary);
    if (!in) return fail(error, "Cannot open " + path);
    points->resize(size / sizeof(LidarPoint));
    in.read(reinterpret_cast<char*>(points->data()), static_cast<std::streamsize>(size));
    if (!in) return fail(error, "Short read: " + path);
    return true;
}

// ---- playback ------------------------------------------------------------------------------------------------------

void RecordedPlayback::setDataset(std::shared_ptr<const RecordedDataset> data) {
    data_ = std::move(data);
    time_ = 0.0;
    playing_ = false;
}

void RecordedPlayback::play() {
    if (!loaded()) return;
    if (time_ >= data_->duration()) time_ = 0.0;  // restart from the beginning when at the end
    playing_ = true;
}

void RecordedPlayback::setRate(double rate) { rate_ = std::min(std::max(rate, 0.1), 8.0); }

void RecordedPlayback::advance(double dt) {
    if (!playing_ || !loaded() || !(dt > 0.0)) return;
    time_ += dt * rate_;
    if (time_ >= data_->duration()) {
        time_ = data_->duration();
        playing_ = false;
    }
}

void RecordedPlayback::seek(double t) {
    if (!loaded()) return;
    time_ = std::min(std::max(t, 0.0), data_->duration());
}

void RecordedPlayback::seekFrame(int index) {
    if (!loaded()) return;
    const int n = static_cast<int>(data_->frames.size());
    time_ = data_->frames[static_cast<std::size_t>(std::min(std::max(index, 0), n - 1))].t;
}

void RecordedPlayback::stepFrame(int delta) {
    if (!loaded()) return;
    seekFrame(frameIndex() + delta);
}

int RecordedPlayback::frameIndex() const {
    if (!loaded()) return 0;
    const auto& fr = data_->frames;
    const auto it = std::upper_bound(fr.begin(), fr.end(), time_ + 1e-9, [](double t, const RecordedFrame& f) { return t < f.t; });
    return it == fr.begin() ? 0 : static_cast<int>(it - fr.begin()) - 1;
}

const RecordedFrame* RecordedPlayback::frame() const { return loaded() ? &data_->frames[static_cast<std::size_t>(frameIndex())] : nullptr; }

double RecordedPlayback::nearestAhead(const RecordedFrame& frame, double halfWidth) {
    double best = -1.0;
    for (const RecordedObject& o : frame.objects) {
        const double halfObj = 0.5 * std::max(o.width, 0.0);
        if (o.x <= 0.0 || std::abs(o.y) >= halfWidth + halfObj) continue;
        const double front = o.x - 0.5 * o.length;  // near edge, approximately
        const double d = std::max(front, 0.0);
        if (best < 0.0 || d < best) best = d;
    }
    return best;
}

}  // namespace driveforge
