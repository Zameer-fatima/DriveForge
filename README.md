# DriveForge

Educational 3D autonomous-car simulator and ADAS test bench (C++17, Qt 6, Qt Quick 3D, CMake, Linux).
**Not a safety-certified automotive system.** All displayed data is *simulated* unless a future
recorded-data mode explicitly labels it as real-world.

See [docs/DESIGN.md](docs/DESIGN.md) for architecture, models, and the roadmap.

## Status

| Milestone | State |
|-----------|-------|
| M1: 3D scene, follow camera, manual controls, vehicle dynamics, basic dashboard, unit tests | **done** |
| M2: telemetry graphs (Qt Charts, 30 s window) + event log | **done** |
| M3: simulated sensors, AEB, ACC, lane keeping, lane change, Autonomous mode, test objects | **done** |
| M4: parking lot, bay detection, reverse trajectory planner, automated perpendicular parking | **done** |
| M5: scenario test bench (highway, obstacle, pedestrian, parking), parameters, repeatability check, reports, headless CLI | **done** |
| M6: recorded real-world data replay (KITTI raw): GPS/IMU, camera, lidar, annotations | **done** |

Everything in the simulator is simulated. Recorded real-world data lives in a separate, clearly labelled mode (F3).

## Requirements

- Linux with OpenGL (Mesa is fine), CMake ≥ 3.21, Ninja, a C++17 compiler
- Qt ≥ 6.5 with modules: Quick, Quick 3D, Quick Controls 2, Charts, Widgets, Test (developed against Qt 6.7.3 from the Qt online installer)

## Build, test, run

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$HOME/Qt/6.7.3/gcc_64
cmake --build build
ctest --test-dir build --output-on-failure
./build/driveforge
```

If CMake is older than 3.21, use `~/Qt/Tools/CMake/bin/cmake`.
Optional: `./build/driveforge --screenshot out.png --screenshot-delay 2500` saves a screenshot and exits.
Other options: `--quality low|medium|high`, `--camera front|rear|side|top|closeup` (see *Look and feel* below).

## Controls

| Key | Action |
|-----|--------|
| I | Ignition on/off (throttle is ignored while off) |
| R | Drive / Reverse (only takes effect when stopped) |
| W / ↑ | Throttle |
| S / ↓ / Space | Brake |
| A, D / ←, → | Steer |
| P | Pause / resume |
| T | Show / hide graphs and event log |
| M | Autonomous mode on/off (ACC 80 km/h + lane keeping + AEB + overtaking; braking or steering = driver takeover) |
| U | Autonomous overtaking on/off (in Autonomous mode: pass slower cars, change lane around obstacles) |
| Mouse | Drag = orbit the camera round the car, wheel = zoom, double-click or Home = reset the camera |
| C | ACC on/off (sets speed to current, min 30 km/h); `+` / `-` change set speed 5 km/h; `G` cycles time gap 1.2 / 1.8 / 2.4 s |
| K | Lane keeping on/off (yields while you steer, re-centres afterwards) |
| Q / E | Lane change left / right (needs lane keeping; refused if the target lane is occupied) |
| B | AEB armed / disabled |
| O / H / V / X | Spawn obstacle / pedestrian / lead car (50 km/h) ahead; clear all (also removes the lot) |
| L | Add a parking lot ahead (8 bays on the right, bays 4 and 7 free) |
| F2 | Scenario test bench (pick, configure, run, report). **Esc** aborts a running scenario |
| F3 | Recorded real-world data view (pauses the simulator; Space play/pause, ←/→ step, F3 or Esc back) |
| F4 | Cycle graphics quality (low / medium / high) |
| N | Parking assist on/off (searches for a free bay, shows the planned path) |
| Y | Park: switches the assist on if needed, waits up to 10 s for a bay, then parks (brake/steer/throttle cancels) |
| Backspace | Reset |

## Look and feel

The 3D view aims for a realistic look using free assets and code only:

- **Cars** are generated procedurally: a smooth lofted body with glossy clear-coated paint, glass, painted pillars, flush headlamps, tail lamps and indicators,
  and wheels with tyres, alloy rims and brake discs. Three body styles (sedan, SUV, van) and several paint colours are used for traffic and parked cars.
- **The car reacts**: it pitches under braking and acceleration, leans in corners, shows brake and reversing lights, blinks when changing lanes,
  and flashes its hazards during emergency braking.
- **People and objects**: pedestrians are articulated and walk with swinging arms and legs; obstacles are road barriers with a flashing lamp.
- **The world** has a photographic asphalt and grass, guardrails, speed-limit signs, pine and broadleaf trees, distant hills, an HDR sky with real
  clouds (which also lights the scene and is reflected in the paint and glass), haze, bloom, ambient occlusion and soft shadows.
- **Performance**: 60 fps while driving at high quality on the development GPU (about 40 fps in the busiest view, the parking lot with eight cars).
  Press **F4** (or start with `--quality low`) to trade detail for speed.
- The textures and the sky are CC0 (public domain), credited in `assets/LICENSES.md`. Everything else in the scene is generated.

## Layout

```
src/core/   Qt-free simulation core — unit tested: VehicleModel, World, Sensors, Aeb, Acc, LaneKeeping, Parking, Adas, Scenario, ScenarioRunner, Recorded,
            Simulation (owns the step order), Telemetry, EventDetector, RingBuffer
src/app/    SimulationController (fixed 120 Hz step, QML facade), EventLogModel, WorldModel, BayModel, CarGeometry (procedural car body), ScenarioManager, RecordedData (+ image provider), main.cpp (also the headless CLI)
qml/        Main, Scene3D, Car, VehicleBody, Pedestrian3D, Tree3D, Dashboard, TelemetryPanel/Chart, EventLog, ScenarioPanel, RecordedView/Chart (presentation only, no ADAS logic)
tests/      QtTest unit tests
```

## Vehicle model (summary)

Kinematic bicycle model with rate-limited pedals and steering; steering is capped by a lateral-acceleration
limit at speed. Braking distance shown on the dashboard is `v² / (2·a_brake)` and ignores drag and pedal ramp-up.
Valid for dry-road, low-slip driving only. Equations and parameters: `docs/DESIGN.md`, `src/core/VehicleModel.h`.

## ADAS (summary; details in docs/DESIGN.md)

- **Sensors** are simulated and ideal (no noise): a forward radar (150 m ahead, 20 m behind) and a lane camera.
- **AEB** brakes on required deceleration `v²/2gap`, on time-to-collision < 1 s, or when very close; it predicts crossing
  pedestrians, cuts the throttle near a target, and holds the brake at standstill while you keep accelerating.
- **ACC** holds a set speed or follows a lead vehicle/obstacle at `5 m + time gap × speed`, down to standstill.
- **Lane keeping** uses a Stanley controller with a comfort limit of 2 m/s² lateral acceleration; **lane change** is a 4 s cosine manoeuvre.
- **Autonomous overtaking** (Autonomous mode only): the car overtakes a slower vehicle in a free lane and returns to its own lane. If a stationary
  obstacle appears ahead, it changes lane around it at speed. If there is no free lane, or it is too close to swerve, it slows or brakes to a stop instead.
  It uses the same safety rules as a manual lane change, reacts to a new obstacle even in the middle of an overtake, and returns to the lane it started in.
  Press **U** to switch it off (the car then follows slower traffic and stops for obstacles).
- Not modelled: sensor noise or dropouts, weather, tyre slip, other vehicles reacting to you, traffic rules. Do not treat
  tuning here as meaningful for real vehicles.

## Parking assist (summary; details in docs/DESIGN.md)

Perpendicular reverse-in parking into bays on the **right** side of the road.

1. Press **L** (lot ahead), drive to within 40 m of the bays in the right-hand part of the road, heading straight.
2. Press **N**. Free bays turn green; when a collision-free plan exists the best bay turns blue, its planned path is drawn, and the dashboard says "ready".
3. Press **Y** (any normal driving speed). The car aligns, turns the wheels while stopped, reverses along the arc, straightens, and backs into the bay.
   Braking, steering, or throttle cancels. Ignition off, an obstacle on the path, or a timeout abort it.

Limits: right-side perpendicular bays only (no parallel parking), start with heading roughly parallel to the road, no moving traffic.

## Scenarios and reports

Press **F2** (or the Scenarios button). Choose Highway, Obstacle, Pedestrian, Parking, Overtaking, or Lane avoid, adjust the sliders, and:

- **Run live** watches the scenario in the 3D scene (1–8× speed). The driver is scripted, so manual controls are locked; **Esc** aborts.
- **Repeat 3× headless** runs it three times without rendering and checks that every run gives identical results.
- **Export report** writes `<scenario>-<time>.md` and `.json` to `~/.local/share/DriveForge/reports/`.

Each report lists the parameters, every pass/fail criterion with the required and measured value, and the metrics.
Obstacle and pedestrian runs with **AEB off** are expected to fail (they show what AEB prevents).

### Headless (scripts / CI)

```bash
./build/driveforge --list-scenarios
./build/driveforge --run-scenario pedestrian --set speed=70 --repeat 3 --report-dir reports
./build/driveforge --run-scenario obstacle --set aebArmed=0 --json     # exit code 1: fails
```

Exit code: 0 = every run passed and repeats were identical, 1 = a failure or differing repeats, 2 = bad arguments.

## Recorded data (real-world, separate from the simulator)

Press **F3** to replay a real recording in the [KITTI](https://www.cvlibs.net/datasets/kitti/) raw format: GPS/IMU speed, acceleration, yaw rate and path,
the left colour camera, a top-down Velodyne lidar view, and the dataset's human-annotated object boxes. It is a viewer: it does not drive the
simulator, the ADAS, or the scenarios. Everything is labelled **RECORDED**; nothing is generated or interpolated, and missing data stays missing.

```bash
./scripts/fetch_kitti.sh                          # one-time: 2011-09-26 drive 0001, ~460 MB download, ~320 MB on disk, into data/kitti/
./build/driveforge --recorded-info data/kitti  # what the loader found (frames, duration, sensors, warnings)
./build/driveforge --recorded data/kitti       # open the app with the recording loaded
```

Or press F3 in the app and use Browse / Load on any KITTI raw drive folder (the folder that contains `oxts/`, optionally `image_02/`,
`velodyne_points/` and `tracklet_labels.xml`; a parent folder works too).

- **Licence:** KITTI is CC BY-NC-SA 3.0 (attribution, non-commercial, share-alike). It is not part of this repository and must not be redistributed with it.
- **Citation:** A. Geiger, P. Lenz, C. Stiller, R. Urtasun, "Vision meets Robotics: The KITTI Dataset", International Journal of Robotics Research, 2013.
- **Not available in this data:** steering angle, brake, throttle. Object boxes are human annotations, not sensor detections (dashed = interpolated by the annotation tool).
- The unit tests use small synthetic files in the KITTI format (generated at test time, never shown in the app). With the real drive downloaded to `data/kitti`,
  `ctest` also runs a test against it (set `DRIVEFORGE_KITTI_DIR` to use another location).
