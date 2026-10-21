import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import DriveForge

// Live readout of simulated vehicle state. All values here are SIMULATED.
Rectangle {
    id: root
    required property SimulationController sim

    implicitWidth: 560
    implicitHeight: 235
    radius: 10
    color: "#cc10141a"
    border.color: "#33ffffff"

    // Avoid showing "-0.00" for tiny negative values.
    function fmt(v, digits) { return (Math.abs(v) < 0.5 * Math.pow(10, -digits) ? 0 : v).toFixed(digits) }

    component Stat: ColumnLayout {
        property string label
        property string value
        spacing: 0
        Label { text: parent.label; color: "#8fa3b8"; font.pixelSize: 11 }
        Label { text: parent.value; color: "#e8eef5"; font.pixelSize: 20; font.family: "monospace" }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 8

    RowLayout {
        Layout.fillWidth: true
        spacing: 22

        ColumnLayout {
            spacing: 0
            Label {
                text: Math.abs(root.sim.speedKmh).toFixed(0)
                color: "white"; font.pixelSize: 56; font.bold: true; font.family: "monospace"
                Layout.alignment: Qt.AlignHCenter
            }
            Label { text: "km/h"; color: "#8fa3b8"; Layout.alignment: Qt.AlignHCenter }
            Label {
                text: !root.sim.ignition ? "OFF"
                    : (root.sim.reverse !== root.sim.reverseRequested
                       ? (root.sim.reverseRequested ? "D→R" : "R→D")   // waiting for the car to stop
                       : (root.sim.reverse ? "R" : "D"))
                color: root.sim.reverse ? "#ff9f43" : "#4cd964"
                font.pixelSize: 22; font.bold: true
                Layout.alignment: Qt.AlignHCenter
            }
        }

        GridLayout {
            columns: 2
            columnSpacing: 28
            rowSpacing: 6
            Stat { label: "ACCELERATION"; value: root.fmt(root.sim.acceleration, 2) + " m/s²" }
            Stat { label: "STEERING"; value: root.fmt(root.sim.steeringDeg, 1) + "°" }
            Stat { label: "BRAKING DISTANCE"; value: root.sim.brakingDistance.toFixed(1) + " m" }
            Stat { label: "OBSTACLE DISTANCE"; value: root.sim.frontTarget ? root.sim.frontGap.toFixed(1) + " m" : "--" }
        }

        ColumnLayout {
            Layout.fillWidth: true
            spacing: 4
            Label { text: "THROTTLE"; color: "#8fa3b8"; font.pixelSize: 11 }
            ProgressBar { value: root.sim.throttle; Layout.fillWidth: true }
            Label { text: "BRAKE"; color: "#8fa3b8"; font.pixelSize: 11 }
            ProgressBar { value: root.sim.brake; Layout.fillWidth: true }
            Label { text: "All sensor data is SIMULATED"; color: "#6c7f93"; font.pixelSize: 10; font.italic: true }
        }
    }

    // ADAS and sensor status strip
    GridLayout {
        Layout.fillWidth: true
        columns: 3
        columnSpacing: 18
        rowSpacing: 2

        component Chip: Label {
            property color tone: "#8fa3b8"
            color: tone
            font.pixelSize: 12
            font.family: "monospace"
        }

        Chip {
            text: "MODE " + (root.sim.autonomous ? "AUTONOMOUS" : "MANUAL")
            tone: root.sim.autonomous ? "#4cc3ff" : "#c9d6e3"
        }
        Chip {
            text: !root.sim.aebArmed ? "AEB OFF" : root.sim.aebBraking ? "AEB BRAKING" : root.sim.aebWarning ? "AEB WARNING" : "AEB ready"
            tone: !root.sim.aebArmed ? "#ff5d5d" : root.sim.aebBraking ? "#ff5d5d" : root.sim.aebWarning ? "#ffb454" : "#4cd964"
        }
        Chip {
            text: "RADAR " + root.sim.radarStatus
            tone: root.sim.frontTarget ? "#ffb454" : "#8fa3b8"
        }
        Chip {
            text: root.sim.accEnabled
                  ? "ACC " + root.sim.accSetSpeedKmh.toFixed(0) + " km/h · gap " + root.sim.accTimeGap.toFixed(1) + " s" + (root.sim.accHasLead ? " · following" : "")
                  : "ACC off"
            tone: root.sim.accEnabled ? "#4cd964" : "#8fa3b8"
        }
        Chip {
            text: !root.sim.lkaEnabled ? "LKA off"
                  : root.sim.lkaOverridden ? "LKA suspended (driver)"
                  : root.sim.laneChanging ? "LKA lane change…"
                  : "LKA centring · off " + root.sim.laneOffset.toFixed(2) + " m"
            tone: !root.sim.lkaEnabled ? "#8fa3b8" : root.sim.lkaOverridden ? "#ffb454" : "#4cd964"
        }
        Chip {
            Layout.columnSpan: 3
            text: root.sim.parkingEnabled ? "PARKING " + root.sim.parkingText : "PARKING assist off"
            tone: root.sim.parkingActive ? "#4cc3ff" : root.sim.parked ? "#4cd964" : root.sim.parkingReady ? "#4cd964" : "#8fa3b8"
        }
        Chip {
            text: "CAMERA " + root.sim.cameraStatus
            tone: root.sim.cameraStatus.indexOf("no lane") >= 0 ? "#ffb454" : "#8fa3b8"
        }
    }
    }
}
