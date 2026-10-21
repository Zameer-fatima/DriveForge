# DriveForge — Design

Educational 3D autonomous-car simulator and ADAS test bench. **Not a safety-certified automotive system.**
Stack: C++17, Qt 6.7 (Quick/QML, Quick 3D, Charts), CMake. Linux. No paid or hardware dependencies.

## Layers

```
 QML (presentation only)          Main  Scene3D  Car  Dashboard  TelemetryPanel/Chart  EventLog
        │  properties / invokables
 SimulationController (Qt)        QObject facade, fixed-step loop, QML_ELEMENT; list models for events, objects, bays
        │  plain C++ calls
 driveforge_core (no Qt)           Simulation (owns the step order)
                                  VehicleModel  World  Sensors  Aeb  Acc  LaneKeeping  Parking  Adas
                                  Telemetry  EventDetector  RingBuffer  Longitudinal
                                  Scenario  ScenarioRunner                Recorded (KITTI loader + replay clock)
```

Rules: `driveforge_core` never includes Qt (unit-testable, deterministic). QML never contains
vehicle or ADAS logic. ADAS reads *sensor* structs and returns *control commands* only; it
never touches vehicle state directly.

## Frame loop

Fixed timestep `dt = 1/120 s`, accumulator driven by real elapsed time (clamped to 0.1 s).
Order per step (`Simulation::step`): world → sensors → ADAS (merges driver input) → `VehicleModel::step` → collisions → events/telemetry.
Same inputs + same dt ⇒ identical results (repeatable scenario runs).

## Coordinate conventions

Sim: x = right, y = forward along road, heading ψ = 0 along +y, positive = clockwise (right turn), SI units.
3D scene: `(x, 0, -y)`, `eulerRotation.y = -ψ`.

## Vehicle model (M1)

Kinematic bicycle model, wheelbase L:

- `ẋ = v sinψ`, `ẏ = v cosψ`, `ψ̇ = (v / L) tanδ`
- Longitudinal: `a_drive = throttle · a_max · (1 − v/v_max)` (reverse: scaled, own v_max);
  resistance `a_res = c_roll + c_drag·v²` and brake `a_brk = brake · min(a_brake_max, μg)` only ever
  reduce |v| toward zero (no sign flip).
- Pedals: rate-limited actuators. Steering: rate-limited, and limited by lateral acceleration
  `|δ| ≤ atan(a_lat_max · L / v²)`.
- Braking distance (reported): `v² / (2 · min(a_brake_max, μg))`, ignores drag and pedal ramp (documented, slightly optimistic).
- Gear change (D/R) only when `|v| < 0.1 m/s`. Ignition off ⇒ throttle forced to 0.

Valid range is low-speed to highway driving on dry road; no tyre slip, load transfer, or suspension.
The model's position is the vehicle centre.

## Roadmap (each milestone builds, runs, and is tested before the next)

| M | Content |
|---|---------|
| 1 | 3D scene, follow camera, manual controls, dynamics, basic dashboard, unit tests — **done** |
| 2 | Telemetry ring buffer (10 Hz, 60 s), Qt Charts graphs, event log (edge-triggered detector) — **done** |
| 3 | World objects + simulated sensors; LKA, lane change, AEB, ACC; autonomous mode — **done** |
| 4 | Parking slots, reverse trajectory planner, automated parking — **done** |
| 5 | Scenarios (highway, obstacle, pedestrian, parking), parameters, repeatability check, pass/fail reports, headless CLI — **done** |
| 6 | Recorded-data mode: KITTI raw loader, timestamp replay of GPS/IMU, camera, lidar and annotations, clearly labelled as real data — **done** |

Simulated data is always labelled "SIMULATED"; recorded data "RECORDED (real-world)". Nothing is fabricated as real.

## Telemetry and events (M2)

`Telemetry` samples `VehicleState` every 0.1 simulated seconds into a `RingBuffer` (600 samples).
`EventDetector` emits edge-triggered events with hysteresis: ignition, gear, moving/stopped,
hard braking (> 5 m/s², re-arms below 2), high lateral acceleration (> 4 m/s², re-arms below 2.5).
`SimulationController::fillSeries` copies a channel into a Qt Charts series; the log is an `EventLogModel` (max 500 entries).
Qt Charts' QML `ChartView` requires `QApplication`, so the app links Qt Widgets.

## World, sensors, ADAS (M3)

**World**: straight 3-lane road (`Road`), axis-aligned boxes (`vehicle`, `obstacle`, `pedestrian`). Pedestrians stay put until the
ego car is within `triggerTtc` seconds, then walk at constant velocity. Contact (SAT test) stops the car dead, logs a collision,
and disengages ADAS.

**Sensors** (ideal, simulated): per object gap (bumper-to-bumper along heading), lateral offset, closing speed, lateral speed;
lane index, offset and heading error; parking bays (M4). Range 150 m ahead, 20 m behind, ±16 m sideways.

**AEB** (`Aeb`): for each in-path detection (constant-velocity prediction: in the corridor when our front arrives, when our rear has passed, or crossing it in between — the car is 4.5 m long, so a walking pedestrian can step into the flank), required deceleration
`a = v_close² / 2 gap`; warning at ≥ 2.5 m/s² or TTC ≤ 2.5 s; braking latched at ≥ 4 m/s², TTC ≤ 1.7 s, or gap < 2 m;
brake demand `1.3 · a / a_max` (min 0.4). Released after 0.5 s without a threat or at standstill; at standstill it keeps holding while
the driver still accelerates towards a target closer than 10 m. Throttle is cut when an in-path target is closer than `3 m + 0.5 s · v`.
Forward driving only (no rear sensing).

**ACC** (`computeAcc`): `a = min(k_set (v_set − v), k_v (v_lead − v) + k_g (gap − (5 + t_gap v)))`, clamped to [−3.5, +2] m/s²,
converted to pedals by inverting the longitudinal model. Lead = vehicle or obstacle within the lane corridor (not pedestrians).

**Lane keeping** (`LaneKeeping`): Stanley `δ = −(ψ_err + atan(k e / (v + k_s)))`, steering capped at 2 m/s² lateral acceleration.
Lane change: reference offset follows `x0 + Δ (1 − cos πt/T) / 2`, T = 4 s, with heading feed-forward; refused if the target lane has an object
closer than `12 m + 2 s · closing` ahead or `10 m + 2.5 s · closing` behind, or there is no lane.

**Modes** (`Adas`): Manual (each assist is optional) or Autonomous (ACC 80 km/h + lane keeping + AEB; driver pedals ignored).
Brake or steering input in Autonomous mode is a takeover back to Manual. In Manual, driver braking cancels ACC and driver steering suspends
lane keeping. Ignition off, reverse gear, lost lane markings, or a collision disengage assists.

Simplifications: objects are boxes aligned with the road; no sensor noise.

## Parking (M4)

**World**: `ParkingSlot` bays (3.0 m wide along y, 5.5 m deep along x) start 3 m beyond the road edge, open towards the road. Parked cars are
persistent `Vehicle` objects lying across their bay. The simulated side sensors report each bay within 40 m with a `free` flag
(no object overlaps it). Detections also carry world y and half-length so the planner can build collision boxes.

**Geometry** (`ParkingAssist::planFor`): from the car's pose (heading ≈ 0, lateral position x0) reversing with steering `δ = atan(L/R)` to the right
turns the car through 90° along a circle of radius R and ends at (x0 + R, y0 − R) heading −90°. The car then reverses straight by
`S = xf − x0 − R ≥ 0` to the bay centre. So y0 = yf + R. R ranges over `[L / tan(0.9 δmax), xf − x0]`; each candidate path (~0.25 m
spacing) is checked against all detections with the car inflated by 0–0.6 m, and the radius with the most clearance (≥ 0.2 m) wins.

**State machine**: Off → Searching → Ready → Align (straight along the road to y0, forward or reverse) → PreSteer (stopped, wheels to δ) → Arc
(constant steering, speed profile `sqrt(2 a d)` with a crawl floor) → Straighten (stopped) → Straight (heading hold on the line y = yf) →
Settle → Parked. Gear changes only happen at standstill, which is why each segment ends stopped.
Success needs all four car corners inside the bay (0.1 m tolerance) and heading within 5.7° of perpendicular.

**Safety**: the plan ahead (2.5 m) is re-checked against live detections every step; an obstacle aborts with a brake hold. Driver brake/steer/throttle
takeover, ignition off, collision, or a 240 s timeout abort. Starting parking turns off ACC, lane keeping, and autonomous mode.

**Control helpers**: `pedalsForSpeed` inverts the longitudinal model (forward and reverse) for the speed profiles; `headingHold` steers with
`δ = atan(L ψ̇_des / v)`, valid for both directions.

## Scenarios (M5)

`Scenario.h` holds the catalog (name, description, parameters with default / min / max / step) and the report type. `ScenarioRunner`
sets a scenario up on a `Simulation`, scripts the other actors and the driver, tracks ground-truth metrics, ends the run, and evaluates the
criteria. Per step: `beforeStep(sim)` → `sim.step(dt)` → `afterStep(sim)`. `ScenarioRunner::runHeadless` does that on a fresh simulation,
so the GUI's live run and the command line give identical numbers (same code, fixed 1/120 s step, no randomness).

| Scenario | Setup | Criteria |
|---|---|---|
| Highway | Autonomous mode (ACC + lane keeping + AEB) behind a lead car that brakes at a set time | no collision; gap ≥ 2 m; lane offset ≤ 0.5 m; lateral accel ≤ 3 m/s²; autonomous mode stays on |
| Obstacle | Driver holds the throttle towards a static obstacle (own lane, or another lane) | same lane: no collision, clearance ≥ 0.3 m, stopped; other lane: no collision, no false AEB braking |
| Pedestrian | Pedestrian steps out of the right kerb at a set TTC and walks across | no collision; clearance ≥ 0.5 m; deceleration ≤ 9 m/s² |
| Parking | Car finds a free bay and parks | parked in time; no collision; offset ≤ limit; heading within 5°; inside the lines |

Distances are bumper-to-bumper via `separation()` (ground truth, used only for scoring, never by the ADAS).
Parameters are clamped to their ranges. Reports (`ScenarioReport`) render to Markdown and JSON and carry every measured metric;
`sameResultAs` compares outcome, duration and all metrics exactly, which is the repeatability check.

Live runs lock the manual controls (the driver is scripted) and can run at 1–8× speed; Esc aborts. When a run ends, assists are released and the
car brakes moderately. Headless: `driveforge --run-scenario <name> [--set k=v] [--repeat n] [--report-dir d] [--json]`
(exit 0 pass, 1 fail or non-identical repeats, 2 usage error).

Findings from building the scenarios (fixed): AEB braked too late for a pedestrian stepping out at TTC 4 s (TTC trigger 1.0 s → 1.7 s, and the
prediction now covers the whole car length). Remaining legitimate failures are documented by the sweeps: for example 130 km/h towards a 30 km/h
car 40 m ahead needs ~9.6 m/s², more than the 8 m/s² brakes, and 90 km/h with a pedestrian stepping out 4 s ahead passes 0.48 m behind them (< 0.5 m).

## Recorded real-world data (M6)

A **separate mode** (F3) that replays a recording in the KITTI raw format. It does not feed the simulator, the ADAS, or the scenarios: it exists to
look at real sensor and object data next to the simulator, without ever mixing the two.

**Rules.** Nothing is simulated, interpolated, smoothed or filled in: every value on screen comes from a file of the dataset, and a frame is held until
the next recorded timestamp. Missing data stays missing (warnings, "n/a"). The view has its own look (amber banner and frame), the simulator is paused
behind it, and the camera and lidar images carry a burned-in "RECORDED ... (KITTI, real)" label so a screenshot cannot be mistaken for simulation.

**Supported dataset.** KITTI raw drive folders (`oxts/`, optionally `image_02/`, `velodyne_points/`, `tracklet_labels.xml`); the loader also finds a drive
folder up to three levels below the folder it is given. Licensed CC BY-NC-SA 3.0, so it is downloaded by `scripts/fetch_kitti.sh` and never shipped.

**Loader** (`Recorded.cpp`, Qt-free, treats the files as untrusted): strict number parsing (whole token, finite), range checks, strictly increasing
timestamps, per-frame file presence, size limits (text 50 MB, lidar 16 MB, frame/track/pose counts), nesting limit in the XML. Timestamps keep the day
count and seconds-of-day apart so relative times stay precise. GPS positions use the KITTI devkit's Mercator projection around the first fix.
Annotation poses with state 0 (unset) are dropped; state 1 (interpolated by the annotation tool) is kept and drawn dashed.

**What is shown**: GPS/IMU speed, forward acceleration, yaw rate, position accuracy and path; the left colour camera frame; a top-down lidar view with the
annotation boxes; the count of annotated objects and the nearest one ahead. The object boxes are the dataset's *human annotations*, not detections by
any sensor, and the UI says so. Not shown (not measured in this data): steering angle, brake and throttle.

**Replay** (`RecordedPlayback`): play/pause, 0.1–8× rate, seek, frame stepping, auto-stop at the end. The Qt side (`RecordedData`) exposes it to QML and
`RecordedImageProvider` renders `image://recorded/camera|lidar/<serial>/<frame>` from the loader's own file list only.

Command line: `driveforge --recorded <dir>` opens the view with a drive loaded; `--recorded-info <dir>` prints a summary headless.

## Visuals

**Vehicles** are generated, not loaded: `CarBodyGeometry` (C++, `QQuick3DGeometry`) lofts a smooth body from side-profile curves (monotone cubic
through control points) and a superellipse cross-section, in three styles (sedan, SUV, van). The lower body and the greenhouse are separate meshes;
each has *subsets* so one mesh carries several materials: paint, dark lower trim, headlamps, tail lamps, left and right indicators and reversing
lamps (lamps are zones of the body surface, so they sit flush), and for the greenhouse painted roof/pillars/sills versus glass. Zones are defined on the
mesh grid (station and ring angle) so their edges follow grid lines. Wheels (tyre, alloy rim, spokes, brake disc, hub cap), mirrors, door lines, grille
and plates are primitives (`VehicleBody.qml`). The triangle winding is counter-clockwise from outside, so front faces and normals point out.

**Driving cues from the simulation**: the body pitches and rolls on its suspension from the smoothed longitudinal and lateral acceleration (nose dips
under braking, leans out of corners); brake lights brighten when braking; reversing lamps; indicators blink during lane changes; hazards flash during AEB braking.

**Other actors**: pedestrians are articulated (limbs swing with the distance walked, facing their direction of travel); obstacles are orange-and-white
road barriers with a flashing lamp; other cars reuse `VehicleBody` with a body style and colour chosen from the object id.

**World**: the road is built from 24 m tiles that are recycled as the car drives, so photographic textures stay fixed to the world instead of sliding.
Asphalt and grass use CC0 colour, normal and roughness maps (see `assets/LICENSES.md`); lane markings, guardrails, speed-limit signs, pine and broadleaf
trees and distant hills are generated. A CC0 HDR sky is the skybox and the image-based light (it also gives the paint and glass their reflections),
with depth haze, filmic tone mapping, bloom, ambient occlusion and soft shadows.

**Quality**: `--quality low|medium|high` (F4 cycles it in the app) scales anti-aliasing (MSAA or FXAA), ambient occlusion, bloom, shadow-map size and filtering,
and the detail of background cars. Qt Quick only renders when something changes, so the on-screen fps figure is only meaningful while the scene moves
(60 fps driving on this GPU at high quality; 40 fps in the busiest view, the parking lot with eight cars, at high; 60 on low).
`--camera front|rear|side|top|closeup` places the camera for screenshots.
