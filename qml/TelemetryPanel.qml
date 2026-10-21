import QtQuick
import QtQuick.Layouts
import DriveForge

// Right-hand side panel: telemetry graphs on top, event log below. Everything shown is SIMULATED.
ColumnLayout {
    id: panel
    required property SimulationController sim
    spacing: 6

    TelemetryChart {
        sim: panel.sim; title: "Speed (km/h)"
        channel: 0; yMin: -20; yMax: 160
        Layout.fillWidth: true; Layout.preferredHeight: 105
    }
    TelemetryChart {
        sim: panel.sim; title: "Acceleration: longitudinal (blue) / lateral (orange), m/s²"
        channel: 1; channel2: 2; yMin: -10; yMax: 8
        Layout.fillWidth: true; Layout.preferredHeight: 105
    }
    TelemetryChart {
        sim: panel.sim; title: "Steering angle (°)"
        channel: 3; yMin: -40; yMax: 40; color1: "#7ee081"
        Layout.fillWidth: true; Layout.preferredHeight: 105
    }
    TelemetryChart {
        sim: panel.sim; title: "Distance to object ahead (m, 150 = none)"
        channel: 6; yMin: 0; yMax: 160; color1: "#ff7a7a"
        Layout.fillWidth: true; Layout.preferredHeight: 105
    }
    EventLog {
        sim: panel.sim
        Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 120
    }
}
