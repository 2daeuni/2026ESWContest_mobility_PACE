import QtQuick
import "."

// DEMO 주행 중 앞차와 충돌했을 때
Item {
    id: ov
    property bool active: false
    property bool demo: true
    property bool carla: false        // DEMO · CARLA 에서 실제 충돌
    property string withWhat: ""
    visible: opacity > 0
    opacity: active ? 1 : 0
    Behavior on opacity { NumberAnimation { duration: 150 } }

    Rectangle { anchors.fill: parent; color: "black"; opacity: 0.5 }
    MouseArea { anchors.fill: parent; enabled: ov.demo }

    Rectangle {
        anchors.centerIn: parent
        width: 620; height: 340
        radius: 28
        color: Theme.card
        border.color: Theme.red
        border.width: 4

        Column {
            anchors.centerIn: parent
            spacing: 16
            Glyph { anchors.horizontalCenter: parent.horizontalCenter; kind: "warn"; size: 64; color: Theme.red }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "충돌 발생"
                font.pixelSize: 52
                font.weight: Font.Black
                color: Theme.red
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: ov.carla ? "DEMO · CARLA 주행 중 충돌했습니다" + (ov.withWhat ? " (" + ov.withWhat + ")" : "")
                              : "DEMO 주행 · 전방 차량과 충돌했습니다"
                font.pixelSize: 20
                color: Theme.sub
            }
            Rectangle {
                visible: ov.demo
                anchors.horizontalCenter: parent.horizontalCenter
                width: 220; height: 58; radius: 16
                color: Theme.text
                Text {
                    anchors.centerIn: parent
                    text: "RESET (R)"
                    font.pixelSize: 22
                    font.weight: Font.Bold
                    color: "white"
                }
                MouseArea { anchors.fill: parent; onClicked: hmi.demoReset() }
            }
        }
    }
}
