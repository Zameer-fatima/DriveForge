import QtQuick
import QtCharts
import DriveForge

// Rolling-window line chart fed from SimulationController telemetry (simulated data).
ChartView {
    id: chart
    required property SimulationController sim
    property int channel: 0            // primary series, see driveforge::TelemetryChannel
    property int channel2: -1          // optional second series
    property string unit: ""
    property real yMin: -1
    property real yMax: 1
    property real windowSeconds: 30
    property color color1: "#4cc3ff"
    property color color2: "#ff9f43"

    antialiasing: true
    legend.visible: false
    backgroundColor: "#cc10141a"
    backgroundRoundness: 8
    margins { left: 0; right: 4; top: 0; bottom: 0 }
    titleColor: "#c9d6e3"
    titleFont.pixelSize: 12

    ValueAxis {
        id: axisX
        min: Math.max(0, chart.sim.simTime - chart.windowSeconds)
        max: Math.max(chart.windowSeconds, chart.sim.simTime)
        tickCount: 4
        labelsVisible: false
        labelsColor: "#8fa3b8"
        gridLineColor: "#22ffffff"
    }
    ValueAxis {
        id: axisY
        min: chart.yMin
        max: chart.yMax
        tickCount: 3
        labelFormat: "%.0f"
        labelsFont.pixelSize: 10
        labelsColor: "#8fa3b8"
        gridLineColor: "#22ffffff"
    }
    LineSeries { id: series1; axisX: axisX; axisY: axisY; color: chart.color1; width: 2 }
    LineSeries { id: series2; axisX: axisX; axisY: axisY; color: chart.color2; width: 2; visible: chart.channel2 >= 0 }

    function refresh() {
        chart.sim.fillSeries(series1, chart.channel)
        if (chart.channel2 >= 0) chart.sim.fillSeries(series2, chart.channel2)
    }
    Connections { target: chart.sim; function onTelemetryUpdated() { chart.refresh() } }
}
