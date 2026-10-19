import QtQuick
import QtQuick.Shapes
import "."

Item {
    id: ring
    property real value: 0            // 0..1
    property var level: null          // 0..3 or null
    property string levelText: "--"
    property string caption: ""
    property int stroke: 14

    readonly property color tint: Theme.levelColor(level)
    width: 160
    height: 160

    Rectangle {   // 바깥 은은한 테두리
        anchors.fill: parent
        anchors.margins: -6
        radius: width / 2
        color: "transparent"
        border.color: Theme.ringOuter
        border.width: 6
    }

    Shape {
        anchors.fill: parent
        preferredRendererType: Shape.CurveRenderer
        ShapePath {
            fillColor: "transparent"
            strokeColor: Theme.track
            strokeWidth: ring.stroke
            capStyle: ShapePath.FlatCap
            PathAngleArc {
                centerX: ring.width / 2; centerY: ring.height / 2
                radiusX: ring.width / 2 - ring.stroke / 2; radiusY: radiusX
                startAngle: 0; sweepAngle: 360
            }
        }
        ShapePath {
            fillColor: "transparent"
            strokeColor: ring.tint
            strokeWidth: ring.stroke
            capStyle: ShapePath.RoundCap
            PathAngleArc {
                id: arc
                centerX: ring.width / 2; centerY: ring.height / 2
                radiusX: ring.width / 2 - ring.stroke / 2; radiusY: radiusX
                startAngle: -90
                sweepAngle: 359.9 * Math.max(0.0, Math.min(1.0, ring.value))
                Behavior on sweepAngle { NumberAnimation { duration: 220; easing.type: Easing.OutCubic } }
            }
        }
    }

    Column {
        anchors.centerIn: parent
        spacing: 2
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            text: ring.levelText
            font.pixelSize: 40
            font.weight: Font.Bold
            color: Theme.text
        }
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            text: ring.caption
            font.pixelSize: 15
            color: Theme.sub
        }
    }
}
