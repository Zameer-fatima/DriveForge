import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import DriveForge

// Scenario test bench: pick a scenario, edit its parameters, run it live or repeat it headless, read and export the report.
// Everything here is SIMULATED.
Rectangle {
    id: root
    required property SimulationController sim
    readonly property ScenarioManager mgr: sim.scenarios

    width: 800
    height: 640
    radius: 10
    color: "#f010141a"
    border.color: "#44ffffff"
    clip: true

    signal closeRequested()

    component Btn: Button { focusPolicy: Qt.NoFocus }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 10

        RowLayout {
            Layout.fillWidth: true
            Label { text: "Scenario test bench"; color: "white"; font.pixelSize: 20; font.bold: true; Layout.fillWidth: true }
            Label { text: "all data SIMULATED"; color: "#8fa3b8"; font.pixelSize: 12 }
            Btn { text: "Close (F2)"; onClicked: root.closeRequested() }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 14

            // ---- left: scenario list, parameters ----
            ColumnLayout {
                Layout.preferredWidth: 440
                Layout.maximumWidth: 440
                Layout.minimumWidth: 440
                Layout.fillHeight: true
                spacing: 8

                RowLayout {
                    Layout.fillWidth: true
                    Repeater {
                        model: root.mgr.catalog
                        delegate: Btn {
                            required property var modelData
                            required property int index
                            Layout.fillWidth: true
                            Layout.minimumWidth: 0
                            Layout.preferredWidth: 0
                            text: modelData.key.charAt(0).toUpperCase() + modelData.key.slice(1)
                            highlighted: root.mgr.selected === index
                            enabled: !root.mgr.running
                            onClicked: root.mgr.selected = index
                        }
                    }
                }
                Label {
                    Layout.fillWidth: true
                    text: root.mgr.description
                    color: "#c9d6e3"; wrapMode: Text.WordWrap; font.pixelSize: 13
                }

                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true
                    ColumnLayout {
                        width: 416
                        spacing: 4
                        Repeater {
                            model: root.mgr.params
                            delegate: ColumnLayout {
                                id: row
                                required property int index
                                required property string label
                                required property string unit
                                required property real value
                                required property real from
                                required property real to
                                required property real step
                                Layout.fillWidth: true
                                spacing: 0
                                RowLayout {
                                    Layout.fillWidth: true
                                    Label { text: row.label; color: "#c9d6e3"; font.pixelSize: 12; Layout.fillWidth: true }
                                    Label {
                                        text: row.value.toFixed(row.step < 1 ? 1 : 0) + (row.unit ? " " + row.unit : "")
                                        color: "white"; font.pixelSize: 12; font.family: "monospace"
                                    }
                                }
                                Slider {
                                    Layout.fillWidth: true
                                    focusPolicy: Qt.NoFocus
                                    enabled: !root.mgr.running
                                    from: row.from; to: row.to; stepSize: row.step
                                    value: row.value
                                    onMoved: root.mgr.setParam(row.index, value)
                                }
                            }
                        }
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    Btn { text: "Defaults"; enabled: !root.mgr.running; onClicked: root.mgr.resetParams() }
                    Label { text: "Speed"; color: "#c9d6e3"; font.pixelSize: 12 }
                    Repeater {
                        model: [1, 2, 4, 8]
                        delegate: Btn {
                            required property int modelData
                            text: modelData + "×"
                            highlighted: root.mgr.timeScale === modelData
                            onClicked: root.mgr.timeScale = modelData
                        }
                    }
                }
                RowLayout {
                    Layout.fillWidth: true
                    Btn {
                        Layout.fillWidth: true
                        text: "▶ Run live"
                        enabled: !root.mgr.running
                        highlighted: true
                        onClicked: { root.mgr.runLive(); root.closeRequested() }
                    }
                    Btn { text: "Abort (Esc)"; enabled: root.mgr.running; onClicked: root.mgr.abort() }
                }
                Btn {
                    Layout.fillWidth: true
                    text: "Repeat 3× headless (repeatability check)"
                    enabled: !root.mgr.running
                    onClicked: root.mgr.runRepeatCheck(3)
                }
            }

            // ---- right: report ----
            Rectangle {
                Layout.fillWidth: true
                Layout.minimumWidth: 280
                Layout.fillHeight: true
                radius: 8
                color: "#22ffffff"

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 6

                    Label { text: "Report"; color: "#c9d6e3"; font.pixelSize: 12 }
                    Label {
                        visible: !root.mgr.hasReport
                        Layout.fillWidth: true
                        text: root.mgr.running ? root.mgr.status : "Run a scenario to get a pass/fail report."
                        color: "#8fa3b8"; wrapMode: Text.WordWrap
                    }
                    Label {
                        visible: root.mgr.hasReport
                        text: root.mgr.outcome
                        font.pixelSize: 38; font.bold: true
                        color: root.mgr.outcome === "PASS" ? "#4cd964" : root.mgr.outcome === "FAIL" ? "#ff5d5d" : "#ffb454"
                    }
                    Label {
                        visible: root.mgr.hasReport
                        Layout.fillWidth: true
                        text: root.mgr.reportTitle + "\n" + root.mgr.summary
                        color: "#e8eef5"; wrapMode: Text.WordWrap; font.pixelSize: 12
                    }
                    ListView {
                        visible: root.mgr.hasReport
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        spacing: 4
                        model: root.mgr.criteria
                        delegate: ColumnLayout {
                            required property var modelData
                            width: ListView.view.width
                            spacing: 0
                            Label {
                                Layout.fillWidth: true
                                text: (modelData.passed ? "✓ " : "✗ ") + modelData.name
                                color: modelData.passed ? "#4cd964" : "#ff5d5d"
                                font.pixelSize: 13; font.bold: true
                                elide: Text.ElideRight
                            }
                            Label {
                                Layout.fillWidth: true
                                text: "   need: " + modelData.requirement + "\n   got:  " + modelData.measured
                                color: "#c9d6e3"; font.pixelSize: 11; wrapMode: Text.WordWrap
                            }
                        }
                    }
                    Label {
                        visible: root.mgr.repeatSummary !== ""
                        Layout.fillWidth: true
                        text: root.mgr.repeatSummary
                        color: root.mgr.repeatSummary.indexOf("DIFFERED") >= 0 ? "#ff5d5d" : "#4cc3ff"
                        wrapMode: Text.WordWrap; font.pixelSize: 12
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Btn { text: "Export report"; enabled: root.mgr.hasReport; onClicked: root.mgr.exportReport() }
                        Label {
                            Layout.fillWidth: true
                            text: root.mgr.exportPath
                            color: "#8fa3b8"; font.pixelSize: 10; elide: Text.ElideMiddle
                        }
                    }
                }
            }
        }
    }
}
