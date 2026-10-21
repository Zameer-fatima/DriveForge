import QtQuick
import QtQuick.Controls
import DriveForge

Rectangle {
    id: root
    required property SimulationController sim
    radius: 8
    color: "#cc10141a"
    clip: true

    Label {
        id: header
        x: 10; y: 6
        text: "Event log (simulated)"
        color: "#c9d6e3"; font.pixelSize: 12
    }

    ListView {
        id: list
        anchors { left: parent.left; right: parent.right; top: header.bottom; bottom: parent.bottom; margins: 8; topMargin: 4 }
        model: root.sim.events
        clip: true
        boundsBehavior: Flickable.StopAtBounds
        ScrollBar.vertical: ScrollBar {}
        onCountChanged: Qt.callLater(list.positionViewAtEnd)

        delegate: Row {
            required property string time
            required property bool warning
            required property string message
            spacing: 8
            Label { text: parent.time; color: "#6c7f93"; font.pixelSize: 12; font.family: "monospace" }
            Label {
                text: (parent.warning ? "⚠ " : "") + parent.message
                color: parent.warning ? "#ffb454" : "#e8eef5"
                font.pixelSize: 12
            }
        }
    }
}
