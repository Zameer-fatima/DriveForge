import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts
import DriveForge

// Full-window view of RECORDED REAL-WORLD driving data (KITTI). Deliberately looks different from the simulator
// (amber banner and frame) and labels every image and value as recorded. Nothing in it is simulated.
Rectangle {
    id: root
    required property RecordedData rec
    property string defaultDir: ""
    signal closeRequested()

    color: "#14110a"
    border.color: "#d89a2b"
    border.width: 3

    component Btn: Button { focusPolicy: Qt.NoFocus }
    component Stat: ColumnLayout {
        property string label
        property string value
        spacing: 0
        Label { text: parent.label; color: "#b99a62"; font.pixelSize: 10 }
        Label { text: parent.value; color: "#fff1d6"; font.pixelSize: 17; font.family: "monospace" }
    }

    FolderDialog {
        id: folderDialog
        title: "Choose a KITTI drive folder"
        onAccepted: { pathField.text = selectedFolder.toString().replace("file://", ""); root.rec.load(pathField.text) }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 3
        spacing: 0

        // ---- provenance banner ----
        Rectangle {
            Layout.fillWidth: true
            height: 34
            color: "#c98a1a"
            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 8
                Label {
                    Layout.fillWidth: true
                    text: root.rec.loaded ? "RECORDED REAL-WORLD DATA  ·  " + root.rec.name + "  ·  not simulated"
                                           : "RECORDED REAL-WORLD DATA MODE  ·  no dataset loaded"
                    color: "white"; font.pixelSize: 15; font.bold: true; elide: Text.ElideRight
                }
                Btn { text: "Back to simulator (F3)"; onClicked: root.closeRequested() }
            }
        }

        // ---- nothing loaded ----
        ColumnLayout {
            visible: !root.rec.loaded
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.margins: 30
            spacing: 10
            Item { Layout.fillHeight: true }
            Label { text: "Replay a recorded drive"; color: "#fff1d6"; font.pixelSize: 26; font.bold: true }
            Label {
                Layout.fillWidth: true; Layout.maximumWidth: 760
                wrapMode: Text.WordWrap; color: "#e0cfa8"; font.pixelSize: 14
                text: "This mode replays a real recording in the KITTI raw format: GPS/IMU ego motion, camera images, Velodyne lidar scans and the dataset's " +
                      "human-annotated object labels. It is separate from the simulator, nothing is interpolated or generated, and missing data stays missing.\n\n" +
                      "Choose the folder of a drive (the one containing oxts/, image_02/, velodyne_points/ and tracklet_labels.xml). See the README, section " +
                      "\"Recorded data\", for how to download the free sample."
            }
            RowLayout {
                Layout.fillWidth: true; Layout.maximumWidth: 760
                TextField {
                    id: pathField
                    Layout.fillWidth: true
                    placeholderText: "/path/to/2011_09_26_drive_0001_sync"
                    text: root.defaultDir
                    selectByMouse: true
                    onAccepted: root.rec.load(text)
                }
                Btn { text: "Browse…"; onClicked: folderDialog.open() }
                Btn { text: "Load"; highlighted: true; onClicked: root.rec.load(pathField.text) }
            }
            Label {
                visible: root.rec.error !== ""
                Layout.fillWidth: true; Layout.maximumWidth: 760
                wrapMode: Text.WordWrap; color: "#ff7a7a"; font.pixelSize: 13
                text: "Could not load: " + root.rec.error
            }
            Item { Layout.fillHeight: true }
        }

        // ---- loaded ----
        RowLayout {
            visible: root.rec.loaded
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.margins: 10
            spacing: 12

            // left: camera and lidar
            ColumnLayout {
                Layout.preferredWidth: 560
                Layout.fillHeight: true
                spacing: 8
                Image {
                    Layout.fillWidth: true
                    Layout.preferredHeight: width * 375 / 1242
                    fillMode: Image.PreserveAspectFit
                    cache: false
                    asynchronous: false
                    source: root.rec.cameraSource
                    visible: root.rec.hasCamera
                }
                Image {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    fillMode: Image.PreserveAspectFit
                    cache: false
                    asynchronous: false
                    source: root.rec.lidarSource
                    visible: root.rec.hasLidar
                }
                Label {
                    visible: !root.rec.hasLidar && !root.rec.hasCamera
                    text: "This recording has no camera images or lidar scans."
                    color: "#ffb454"
                }
            }

            // right: numbers, charts, trace
            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 6

                GridLayout {
                    columns: 4
                    columnSpacing: 20
                    rowSpacing: 4
                    Stat { label: "RECORDED SPEED"; value: root.rec.speedKmh.toFixed(1) + " km/h" }
                    Stat { label: "RECORDED ACCEL (fwd)"; value: root.rec.acceleration.toFixed(2) + " m/s²" }
                    Stat { label: "RECORDED YAW RATE"; value: root.rec.yawRateDeg.toFixed(1) + " °/s" }
                    Stat { label: "GPS ACCURACY"; value: root.rec.positionAccuracy.toFixed(2) + " m" }
                    Stat { label: "GPS LAT"; value: root.rec.latitude.toFixed(6) }
                    Stat { label: "GPS LON"; value: root.rec.longitude.toFixed(6) }
                    Stat { label: "ANNOTATED OBJECTS"; value: root.rec.hasObjects ? root.rec.objectCount : "n/a" }
                    Stat { label: "NEAREST AHEAD (label)"; value: root.rec.hasObjects ? (root.rec.nearestAhead >= 0 ? root.rec.nearestAhead.toFixed(1) + " m" : "--") : "n/a" }
                }

                RecordedChart { Layout.fillWidth: true; Layout.preferredHeight: 110; rec: root.rec; title: "Recorded speed (km/h)"; channel: 0; yMin: 0; yMax: 60 }
                RecordedChart { Layout.fillWidth: true; Layout.preferredHeight: 110; rec: root.rec; title: "Recorded forward acceleration (m/s²)"; channel: 1; yMin: -4; yMax: 4 }
                RecordedChart { Layout.fillWidth: true; Layout.preferredHeight: 110; rec: root.rec; title: "Recorded yaw rate (°/s)"; channel: 2; yMin: -30; yMax: 30 }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.minimumHeight: 90
                    radius: 6
                    color: "#cc1a1608"
                    Canvas {
                        id: traceCanvas
                        anchors.fill: parent
                        anchors.margins: 8
                        onPaint: {
                            const ctx = getContext("2d")
                            ctx.reset()
                            const pts = root.rec.trace
                            if (!pts || pts.length < 2) return
                            let x0 = 1e18, x1 = -1e18, y0 = 1e18, y1 = -1e18
                            for (let i = 0; i < pts.length; ++i) {
                                x0 = Math.min(x0, pts[i].x); x1 = Math.max(x1, pts[i].x)
                                y0 = Math.min(y0, pts[i].y); y1 = Math.max(y1, pts[i].y)
                            }
                            const span = Math.max(x1 - x0, y1 - y0, 5)
                            const s = Math.min(width, height - 18) / span
                            const ox = (width - (x1 - x0) * s) / 2, oy = (height - 18 - (y1 - y0) * s) / 2 + 16
                            const px = p => ox + (p.x - x0) * s
                            const py = p => oy + (y1 - p.y) * s
                            ctx.strokeStyle = "#ffb454"; ctx.lineWidth = 2
                            ctx.beginPath()
                            ctx.moveTo(px(pts[0]), py(pts[0]))
                            for (let i = 1; i < pts.length; ++i) ctx.lineTo(px(pts[i]), py(pts[i]))
                            ctx.stroke()
                            const c = root.rec.position
                            ctx.fillStyle = "white"
                            ctx.beginPath(); ctx.arc(px(c), py(c), 5, 0, 2 * Math.PI); ctx.fill()
                            ctx.fillStyle = "#c9b48a"; ctx.font = "11px sans-serif"
                            ctx.fillText("Recorded GPS/IMU path (north up, " + Math.round(span) + " m across)", 2, 11)
                        }
                        Connections { target: root.rec; function onFrameChanged() { traceCanvas.requestPaint() } function onChanged() { traceCanvas.requestPaint() } }
                    }
                }
            }
        }

        // ---- transport ----
        RowLayout {
            visible: root.rec.loaded
            Layout.fillWidth: true
            Layout.leftMargin: 10; Layout.rightMargin: 10; Layout.bottomMargin: 4
            spacing: 8
            Btn { text: "◀"; onClicked: root.rec.stepFrame(-1) }
            Btn { text: root.rec.playing ? "⏸ Pause" : "▶ Play"; highlighted: true; onClicked: root.rec.toggle() }
            Btn { text: "▶"; onClicked: root.rec.stepFrame(1) }
            Repeater {
                model: [0.5, 1, 2, 4]
                delegate: Btn {
                    required property real modelData
                    text: modelData + "×"
                    highlighted: root.rec.rate === modelData
                    onClicked: root.rec.setRate(modelData)
                }
            }
            Slider {
                Layout.fillWidth: true
                focusPolicy: Qt.NoFocus
                from: 0; to: Math.max(root.rec.duration, 0.1)
                value: root.rec.time
                onMoved: root.rec.seek(value)
            }
            Label {
                text: "frame " + (root.rec.frame + 1) + "/" + root.rec.frameCount + "  ·  recorded t = " + root.rec.recordedTime.toFixed(1) + " s of " + root.rec.duration.toFixed(1) + " s"
                color: "#fff1d6"; font.family: "monospace"; font.pixelSize: 12
            }
        }
        Label {
            visible: root.rec.loaded
            Layout.fillWidth: true
            Layout.leftMargin: 10; Layout.rightMargin: 10; Layout.bottomMargin: 6
            wrapMode: Text.WordWrap
            color: "#b99a62"; font.pixelSize: 10
            text: root.rec.source + ".  License: " + root.rec.license + ".  Object boxes are the dataset's human annotations (not sensor detections); " +
                  "dashed = interpolated by the annotation tool.  Space: play/pause, ←/→: step, F3 or Esc: back." +
                  (root.rec.warnings.length ? "  Notes: " + root.rec.warnings.join("; ") + "." : "")
        }
    }
}
