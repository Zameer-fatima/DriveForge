import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import DriveForge

ApplicationWindow {
    id: window
    width: 1280
    height: 800
    visible: true
    title: "DriveForge — educational simulator (not safety-certified)"

    SimulationController {
        id: sim
        onNoticePosted: text => {
            window.noticeText = text
            noticeTimer.restart()
        }
    }

    // Frame rate readout (shown in the help text)
    property int fps: 0
    property int frameCounter: 0
    Timer { interval: 1000; running: true; repeat: true; onTriggered: { window.fps = window.frameCounter; window.frameCounter = 0 } }
    Connections { target: window; function onFrameSwapped() { window.frameCounter++ } }

    // Recorded real-world data (KITTI replay) is a separate mode. The simulator is paused while it is shown.
    RecordedData { id: recorded }
    property bool pausedForRecorded: false
    function showRecorded(show) {
        if (show === recordedView.visible) return
        if (show) {
            scenarioPanel.visible = false
            if (sim.running) { sim.togglePause(); pausedForRecorded = true }
        } else {
            recorded.pause()
            if (pausedForRecorded && !sim.running) sim.togglePause()
            pausedForRecorded = false
        }
        recordedView.visible = show
    }
    Component.onCompleted: {
        if (startupRecordedDir !== "") { recorded.load(startupRecordedDir); showRecorded(true) }
    }

    // Latest warning (parking refused/aborted, AEB, collision, ...), shown for 7 s.
    property string noticeText: ""
    Timer { id: noticeTimer; interval: 7000 }

    // --- keyboard driving -------------------------------------------------
    property var pressed: ({})
    function isDown(k) { return pressed[k] === true }
    function pushInput() {
        const throttle = (isDown(Qt.Key_W) || isDown(Qt.Key_Up)) ? 1 : 0
        const brake = (isDown(Qt.Key_S) || isDown(Qt.Key_Down) || isDown(Qt.Key_Space)) ? 1 : 0
        const steer = ((isDown(Qt.Key_D) || isDown(Qt.Key_Right)) ? 1 : 0)
                    - ((isDown(Qt.Key_A) || isDown(Qt.Key_Left)) ? 1 : 0)
        sim.setDriverInput(throttle, brake, steer)
    }

    Item {
        id: keys
        anchors.fill: parent
        focus: true
        Keys.onPressed: event => {
            if (event.isAutoRepeat) return
            if (recordedView.visible) {   // the recorded-data view has its own keys; the simulator is paused behind it
                switch (event.key) {
                case Qt.Key_Space: recorded.toggle(); break
                case Qt.Key_Left: recorded.stepFrame(-1); break
                case Qt.Key_Right: recorded.stepFrame(1); break
                case Qt.Key_F3: case Qt.Key_Escape: window.showRecorded(false); break
                }
                event.accepted = true
                return
            }
            switch (event.key) {
            case Qt.Key_I: sim.toggleIgnition(); break
            case Qt.Key_R: sim.toggleGear(); break
            case Qt.Key_P: sim.togglePause(); break
            case Qt.Key_T: panel.visible = !panel.visible; break
            case Qt.Key_M: sim.toggleAutonomous(); break
            case Qt.Key_C: sim.toggleAcc(); break
            case Qt.Key_K: sim.toggleLka(); break
            case Qt.Key_B: sim.toggleAeb(); break
            case Qt.Key_U: sim.toggleOvertake(); break
            case Qt.Key_Home: scene.resetCamera(); break
            case Qt.Key_Q: sim.laneChange(-1); break
            case Qt.Key_E: sim.laneChange(1); break
            case Qt.Key_Plus: case Qt.Key_Equal: sim.accSpeed(1); break
            case Qt.Key_Minus: sim.accSpeed(-1); break
            case Qt.Key_G: sim.nextTimeGap(); break
            case Qt.Key_O: sim.spawnObstacle(); break
            case Qt.Key_H: sim.spawnPedestrian(); break
            case Qt.Key_V: sim.spawnLeadCar(50); break
            case Qt.Key_X: sim.clearObjects(); break
            case Qt.Key_N: sim.toggleParkingAssist(); break
            case Qt.Key_F2: scenarioPanel.visible = !scenarioPanel.visible; break
            case Qt.Key_F3: window.showRecorded(true); break
            case Qt.Key_F4: scene.quality = (scene.quality + 1) % 3; break
            case Qt.Key_Escape: sim.scenarios.abort(); break
            case Qt.Key_Y: sim.startParking(); break
            case Qt.Key_L: sim.spawnParkingLot(); break
            case Qt.Key_Backspace: sim.reset(); break
            default:
                window.pressed[event.key] = true
                window.pushInput()
            }
            event.accepted = true
        }
        Keys.onReleased: event => {
            if (event.isAutoRepeat) return
            window.pressed[event.key] = false
            window.pushInput()
            event.accepted = true
        }
    }

    Scene3D {
        id: scene
        anchors.fill: parent
        sim: sim
    }

    // Left controls: drive, ADAS, test objects
    component Btn: Button { focusPolicy: Qt.NoFocus; Layout.fillWidth: true }

    ColumnLayout {
        id: controls
        anchors { left: parent.left; top: parent.top; margins: 14 }
        width: 170
        spacing: 6
        Btn {
            text: sim.ignition ? "Ignition: ON (I)" : "Ignition: OFF (I)"
            highlighted: sim.ignition
            onClicked: sim.toggleIgnition()
        }
        Btn {
            text: (sim.reverseRequested ? "Gear: Reverse (R)" : "Gear: Drive (R)")
                + (sim.reverse !== sim.reverseRequested ? " — waiting" : "")
            onClicked: sim.toggleGear()
        }
        Btn { text: sim.running ? "Pause (P)" : "Resume (P)"; onClicked: sim.togglePause() }
        Btn { text: "Reset (Backspace)"; onClicked: sim.reset() }
        Btn { text: panel.visible ? "Hide graphs (T)" : "Show graphs (T)"; onClicked: panel.visible = !panel.visible }
        Btn { text: "Reset camera (Home)"; onClicked: scene.resetCamera() }
        Btn { text: "Scenarios (F2)"; highlighted: sim.scenarios.running; onClicked: scenarioPanel.visible = !scenarioPanel.visible }
        Btn { text: "Recorded real data (F3)"; onClicked: window.showRecorded(true) }
        Label { text: "DRIVER ASSISTANCE"; color: "#c9d6e3"; font.pixelSize: 11; Layout.topMargin: 8
                style: Text.Outline; styleColor: "#cc000000" }
        Btn { text: sim.autonomous ? "Autonomous: ON (M)" : "Autonomous: off (M)"; highlighted: sim.autonomous; onClicked: sim.toggleAutonomous() }
        Btn { text: sim.accEnabled ? "ACC: ON (C)" : "ACC: off (C)"; highlighted: sim.accEnabled; onClicked: sim.toggleAcc() }
        Btn { text: sim.lkaEnabled ? "Lane keeping: ON (K)" : "Lane keeping: off (K)"; highlighted: sim.lkaEnabled; onClicked: sim.toggleLka() }
        Btn { text: sim.aebArmed ? "AEB: armed (B)" : "AEB: OFF (B)"; highlighted: sim.aebArmed; onClicked: sim.toggleAeb() }
        Btn { text: sim.overtakeEnabled ? "Auto overtake: ON (U)" : "Auto overtake: off (U)"; highlighted: sim.overtakeEnabled; onClicked: sim.toggleOvertake() }
        RowLayout {
            Layout.fillWidth: true
            Btn { text: "◀ Lane (Q)"; enabled: sim.lkaEnabled; onClicked: sim.laneChange(-1) }
            Btn { text: "Lane ▶ (E)"; enabled: sim.lkaEnabled; onClicked: sim.laneChange(1) }
        }
        Btn { text: sim.parkingEnabled ? "Parking assist: ON (N)" : "Parking assist: off (N)"; highlighted: sim.parkingEnabled; onClicked: sim.toggleParkingAssist() }
        Btn { text: "Park in bay (Y)"; onClicked: sim.startParking() }
        Label { text: "TEST OBJECTS (simulated)"; color: "#c9d6e3"; font.pixelSize: 11; Layout.topMargin: 8
                style: Text.Outline; styleColor: "#cc000000" }
        Btn { text: "Obstacle ahead (O)"; onClicked: sim.spawnObstacle() }
        Btn { text: "Pedestrian (H)"; onClicked: sim.spawnPedestrian() }
        Btn { text: "Lead car 50 km/h (V)"; onClicked: sim.spawnLeadCar(50) }
        Btn { text: "Parking lot ahead (L)"; onClicked: sim.spawnParkingLot() }
        Btn { text: "Clear objects (X)"; onClicked: sim.clearObjects() }
    }

    Label {
        anchors { right: parent.right; top: parent.top; margins: 14 }
        horizontalAlignment: Text.AlignRight
        color: "white"
        style: Text.Outline; styleColor: "#80000000"
        text: "W/↑ throttle   S/↓/Space brake   A/D or ←/→ steer   R drive/reverse\n" +
              "M autonomous   U auto-overtake   C ACC (+/- speed, G time gap)   K lane keeping   Q/E lane change   B AEB\n" +
              "O obstacle   H pedestrian   V lead car   L parking lot   X clear   N park assist   Y park   F2 scenarios\n" +
              "Mouse: drag = orbit camera   wheel = zoom   double-click or Home = reset camera\n" +
              "Mode: " + (sim.autonomous ? "AUTONOMOUS" : "MANUAL") + " · data: SIMULATED · " + window.fps + " fps · graphics " + ["low", "medium", "high"][scene.quality] + " (F4)"
              + (sim.collisions > 0 ? " · collisions: " + sim.collisions : "") + (sim.running ? "" : "\nPAUSED")
    }

    Rectangle {
        visible: noticeTimer.running
        anchors { horizontalCenter: parent.horizontalCenter; top: parent.top; topMargin: 10 }
        width: Math.min(noticeLabel.implicitWidth + 36, parent.width - 400); height: 38; radius: 8
        color: "#e0b45a00"
        Label {
            id: noticeLabel
            anchors.centerIn: parent
            width: parent.width - 24
            horizontalAlignment: Text.AlignHCenter
            elide: Text.ElideRight
            text: "⚠ " + window.noticeText
            color: "white"; font.pixelSize: 16; font.bold: true
        }
    }

    Rectangle {
        visible: sim.parked
        anchors { horizontalCenter: parent.horizontalCenter; top: parent.top; topMargin: 105 }
        width: parkedText.implicitWidth + 40; height: 44; radius: 8
        color: "#e01e8a4a"
        Label {
            id: parkedText
            anchors.centerIn: parent
            text: "✓ PARKED — " + sim.parkingText + ".  Press N to switch parking assist off"
            color: "white"; font.pixelSize: 18; font.bold: true
        }
    }

    Rectangle {
        visible: sim.parkingActive
        anchors { horizontalCenter: parent.horizontalCenter; top: parent.top; topMargin: 105 }
        width: parkText.implicitWidth + 40; height: 44; radius: 8
        color: "#e01f5fa8"
        Label {
            id: parkText
            anchors.centerIn: parent
            text: "AUTOMATED PARKING: " + sim.parkingText + "  (brake or steer to cancel)"
            color: "white"; font.pixelSize: 18; font.bold: true
        }
    }

    Rectangle {
        visible: sim.aebWarning
        anchors { horizontalCenter: parent.horizontalCenter; top: parent.top; topMargin: 175 }
        width: aebText.implicitWidth + 40; height: 50; radius: 8
        color: sim.aebBraking ? "#e0d02020" : "#e0e08a00"
        Label {
            id: aebText
            anchors.centerIn: parent
            text: sim.aebBraking ? "EMERGENCY BRAKING (AEB)" : "COLLISION WARNING"
            color: "white"; font.pixelSize: 24; font.bold: true
        }
    }

    Label {
        visible: !sim.ignition
        anchors { horizontalCenter: parent.horizontalCenter; top: parent.top; topMargin: 105 }
        text: "Ignition OFF: press I, then W for throttle"
        color: "#ffd166"; font.pixelSize: 20; font.bold: true
        style: Text.Outline; styleColor: "#cc000000"
    }

    TelemetryPanel {
        id: panel
        sim: sim
        anchors { right: parent.right; top: parent.top; bottom: parent.bottom; topMargin: 90; bottomMargin: 14; rightMargin: 14 }
        width: 380
    }

    Dashboard {
        anchors {
            left: controls.right; leftMargin: 14
            right: panel.visible ? panel.left : parent.right; rightMargin: 14
            bottom: parent.bottom; bottomMargin: 14
        }
        sim: sim
}

    Rectangle {
        visible: sim.scenarios.running
        anchors { horizontalCenter: parent.horizontalCenter; top: parent.top; topMargin: 10 }
        width: scenarioBanner.implicitWidth + 40; height: 36; radius: 8
        color: "#e0245a8f"
        Label {
            id: scenarioBanner
            anchors.centerIn: parent
            text: "SCENARIO  " + sim.scenarios.status + "  ·  " + sim.scenarios.timeScale + "×  ·  Esc to abort"
            color: "white"; font.pixelSize: 15; font.bold: true
        }
    }

    ScenarioPanel {
        id: scenarioPanel
        visible: false
        anchors.centerIn: parent
        width: Math.min(800, parent.width - 40)
        height: Math.min(640, parent.height - 40)
        sim: sim
        onCloseRequested: visible = false
    }

    RecordedView {
        id: recordedView
        visible: false
        anchors.fill: parent
        rec: recorded
        defaultDir: defaultRecordedDir
        onCloseRequested: window.showRecorded(false)
    }
}
