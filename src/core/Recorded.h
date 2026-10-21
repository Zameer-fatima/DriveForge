#pragma once

#include <memory>
#include <string>
#include <vector>

namespace driveforge {

// ---- Recorded REAL-WORLD driving data (KITTI raw format) ------------------------------------------------------------
// Nothing here is simulated or interpolated: every value comes from a file of the dataset. Missing data stays missing.

struct RecordedObject {
    int trackId = 0;
    std::string type;            // human-assigned class label: Car, Van, Truck, Pedestrian, Cyclist, ...
    double x = 0.0, y = 0.0, z = 0.0;  // box bottom centre in the lidar frame (x forward, y left, z up), m
    double yaw = 0.0;            // rad about z (counter-clockwise)
    double length = 0.0, width = 0.0, height = 0.0;
    bool interpolated = false;   // the annotation tool interpolated this pose between hand-labelled frames
};

struct RecordedFrame {
    int index = 0;
    double t = 0.0;              // seconds since the first frame (from the recorded timestamps)

    // GPS/IMU (OXTS). Frame: x east, y north in a local tangent plane around the first fix.
    double latitude = 0.0, longitude = 0.0, altitude = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;  // rad; yaw 0 = east, counter-clockwise positive
    double x = 0.0, y = 0.0;                    // m
    double vForward = 0.0, vLeft = 0.0;         // m/s in the car frame
    double speed = 0.0;                         // m/s, hypot(vForward, vLeft)
    double aForward = 0.0, aLeft = 0.0;         // m/s^2
    double yawRate = 0.0;                       // rad/s about the vertical axis
    double posAccuracy = 0.0;                   // m (as reported by the GPS/IMU)

    std::string cameraPath;      // left colour camera image, empty if the file is missing
    std::string lidarPath;       // Velodyne scan (.bin), empty if the file is missing
    std::vector<RecordedObject> objects;  // human annotations (tracklet labels), NOT sensor detections
};

struct RecordedDataset {
    std::string name;            // e.g. 2011_09_26_drive_0001_sync
    std::string directory;
    std::string source = "KITTI raw data (Geiger, Lenz, Stiller, Urtasun: Vision meets Robotics, IJRR 2013)";
    std::string license = "CC BY-NC-SA 3.0: attribution, non-commercial, share-alike";
    std::vector<RecordedFrame> frames;
    std::vector<std::string> warnings;  // e.g. frames without a camera image
    int trackCount = 0;
    bool hasCamera = false, hasLidar = false, hasObjects = false;

    double duration() const { return frames.empty() ? 0.0 : frames.back().t; }
    double pathLength() const;  // m, summed from the recorded positions
    std::string summary() const;
};

struct LidarPoint {
    float x, y, z, intensity;
};

// Loads a KITTI raw drive directory (the folder containing oxts/, optionally image_02/, velodyne_points/ and
// tracklet_labels.xml). If `dir` is a parent folder, up to three levels below it are searched for a drive folder.
// Returns false with a human-readable `error` if the data is missing, truncated or malformed.
bool loadKittiRaw(const std::string& dir, RecordedDataset* out, std::string* error);

// Reads a Velodyne .bin (float32 x, y, z, intensity per point). Rejects files that are empty, larger than 16 MB, or not a
// multiple of 16 bytes.
bool readVelodyne(const std::string& path, std::vector<LidarPoint>* points, std::string* error);

// Playback clock over a dataset: frames are held (never interpolated) until the next recorded timestamp.
class RecordedPlayback {
public:
    explicit RecordedPlayback(std::shared_ptr<const RecordedDataset> data = nullptr) : data_(std::move(data)) {}

    void setDataset(std::shared_ptr<const RecordedDataset> data);
    bool loaded() const { return data_ && !data_->frames.empty(); }
    const RecordedDataset* dataset() const { return data_.get(); }

    void play();
    void pause() { playing_ = false; }
    void toggle() { playing_ ? pause() : play(); }
    bool playing() const { return playing_; }
    void setRate(double rate);  // clamped to 0.1 .. 8
    double rate() const { return rate_; }

    void advance(double dt);    // wall-clock seconds
    void seek(double t);        // seconds since start, clamped
    void seekFrame(int index);
    void stepFrame(int delta);

    double time() const { return time_; }
    int frameIndex() const;     // the latest frame whose timestamp is <= time()
    const RecordedFrame* frame() const;

    // Nearest annotated object ahead of the car within |lateral| < halfWidth + object half width, or -1.
    static double nearestAhead(const RecordedFrame& frame, double halfWidth = 1.5);

private:
    std::shared_ptr<const RecordedDataset> data_;
    double time_ = 0.0;
    bool playing_ = false;
    double rate_ = 1.0;
};

}  // namespace driveforge
