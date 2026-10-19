import QtQuick
import "."

// 라벨 | 막대 | 값 단위   (첨부 디자인의 PEDAL/HAPTIC/SAFETY 행)
Item {
    id: row
    property string label: ""
    property real ratio: 0              // 0..1
    property string valueText: "--"
    property string unit: ""
    property color barColor: Theme.green
    property color valueColor: Theme.text
    property bool gradient: false       // 주황 그라데이션 (햅틱)
    property int labelWidth: 110
    property int valueWidth: 76

    height: 30
    width: parent ? parent.width : 300

    Text {
        id: lbl
        width: row.labelWidth
        anchors.verticalCenter: parent.verticalCenter
        text: row.label
        font.pixelSize: 16
        color: Theme.text
        elide: Text.ElideRight
    }

    Rectangle {
        id: track
        anchors.left: lbl.right
        anchors.right: val.left
        anchors.rightMargin: 14
        anchors.verticalCenter: parent.verticalCenter
        height: 10
        radius: 5
        color: Theme.track

        Rectangle {
            width: Math.max(row.ratio > 0 ? height : 0, parent.width * Math.max(0, Math.min(1, row.ratio)))
            height: parent.height
            radius: parent.radius
            color: row.barColor
            gradient: row.gradient ? grad : null
            Behavior on width { NumberAnimation { duration: 120 } }
        }

        Gradient {
            id: grad
            orientation: Gradient.Horizontal
            GradientStop { position: 0.0; color: "#FFB15C" }
            GradientStop { position: 1.0; color: Theme.orange }
        }
    }

    Row {
        id: val
        width: row.valueWidth
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        layoutDirection: Qt.RightToLeft
        spacing: 3
        Text {
            text: row.unit
            font.pixelSize: 13
            color: row.valueColor
            anchors.baseline: num.baseline
        }
        Text {
            id: num
            text: row.valueText
            font.pixelSize: 18
            font.weight: Font.DemiBold
            color: row.valueColor
        }
    }
}
