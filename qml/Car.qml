import QtQuick
import QtQuick3D
import DriveForge

// The player's car: VehicleBody plus the motion cues that come from the simulation (suspension pitch and roll, lights).
VehicleBody {
    id: car

    property real accel: 0           // longitudinal, m/s^2 (smoothed)
    property real lateral: 0         // lateral, m/s^2 (smoothed)
    property bool reverse: false
    property int laneChangeDir: 0
    property bool emergency: false

    style: 0
    length: 4.5
    width: 1.85
    paint: "#c4161c"
    reverseLights: reverse
    blinker: laneChangeDir
    hazard: emergency

    // Nose dips under braking and lifts under acceleration; the body leans out of corners.
    pitchDeg: Math.max(-3.5, Math.min(3.5, 0.45 * accel))
    rollDeg: Math.max(-4, Math.min(4, 0.55 * lateral))
    Behavior on pitchDeg { NumberAnimation { duration: 140 } }
    Behavior on rollDeg { NumberAnimation { duration: 140 } }
}
