import QtQuick
import "."

// 왼쪽 클러스터의 세로 페달 게이지 (ACCELERATOR / BRAKE)
Rectangle {
    id: g
    property string title: "ACCELERATOR"
    property var value: null           // 0..100
    property color fill: Theme.green
    property bool alert: false         // 오조작 강조

    color: Theme.cardSoft
    radius: 16
    border.color: alert ? Theme.red : "transparent"
    border.width: alert ? 2 : 0

    Text {
        x: 20; y: 18
        text: g.title
        font.pixelSize: 14
        font.letterSpacing: 0.5
        color: Theme.sub
    }

    Rectangle {
        id: track
        x: 30; y: 58
        width: 26
        height: g.height - 84
        radius: 13
        color: Theme.track

        Rectangle {
            anchors.bottom: parent.bottom
            width: parent.width
            height: Math.max(Theme.has(g.value) && g.value > 0 ? width : 0,
                             parent.height * Theme.clamp01(Theme.has(g.value) ? g.value / 100 : 0))
            radius: parent.radius
            color: g.fill
            Behavior on height { NumberAnimation { duration: 90 } }
        }
    }

    Row {
        anchors.left: track.right
        anchors.leftMargin: 16
        anchors.bottom: track.bottom
        anchors.bottomMargin: -4
        spacing: 2
        Text {
            id: num
            text: Theme.fmt(g.value, 0)
            font.pixelSize: 36
            font.weight: Font.DemiBold
            color: g.alert ? Theme.red : Theme.text
        }
        Text {
            text: "%"
            font.pixelSize: 20
            color: g.alert ? Theme.red : Theme.text
            anchors.baseline: num.baseline
        }
    }
}
