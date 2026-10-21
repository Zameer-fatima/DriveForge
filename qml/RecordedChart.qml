import QtQuick
import QtCharts
import DriveForge

// Whole-recording line chart of one RECORDED channel with a cursor at the current replay time.
ChartView {
    id: chart
    required property RecordedData rec
    property int channel: 0
    property real yMin: -1
    property real yMax: 1

    antialiasing: true
    legend.visible: false
    backgroundColor: "#cc1a1608"
    backgroundRoundness: 6
    margins { left: 0; right: 4; top: 0; bottom: 0 }
    titleColor: "#ffd9a0"
    titleFont.pixelSize: 12

    ValueAxis { id: axisX; min: 0; max: Math.max(1, chart.rec.duration); tickCount: 6; labelFormat: "%.0f"; labelsFont.pixelSize: 10; labelsColor: "#c9b48a"; gridLineColor: "#22ffffff" }
    ValueAxis { id: axisY; min: chart.yMin; max: chart.yMax; tickCount: 3; labelFormat: "%.1f"; labelsFont.pixelSize: 10; labelsColor: "#c9b48a"; gridLineColor: "#22ffffff" }
    LineSeries { id: line; axisX: axisX; axisY: axisY; color: "#ffb454"; width: 2 }
    LineSeries { id: cursor; axisX: axisX; axisY: axisY; color: "#ffffff"; width: 1 }

    function reload() { chart.rec.fillSeries(line, chart.channel); moveCursor() }
    function moveCursor() { chart.rec.fillCursor(cursor, chart.yMin, chart.yMax) }
    Component.onCompleted: reload()
    Connections { target: chart.rec; function onChanged() { chart.reload() } function onFrameChanged() { chart.moveCursor() } }
}
