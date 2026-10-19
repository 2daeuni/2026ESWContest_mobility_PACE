import QtQuick
import "."

// BRAKE NOW 전체 화면 경고 (Risk L3 + 가속 페달 밟는 중 + 브레이크 안 밟음)
// - 화면 전체를 덮어서 무조건 눈에 들어오게 함
// - 운전자가 지금 뭘 해야 하는지(가속 떼기 / 브레이크 밟기)와 현재 페달 값을 같이 보여줌
// - 터치를 막지 않음, 조건이 풀리면 자동으로 사라짐 (Safety ECU 판정만 따름)
Item {
    id: ov
    property bool active: false
    property var s: ({})
    visible: opacity > 0
    opacity: active ? 1 : 0
    Behavior on opacity { NumberAnimation { duration: 120 } }

    readonly property real acc: s.accelerator || 0
    readonly property real brk: s.brake || 0
    readonly property bool accReleased: acc < 5

    // 배경
    Rectangle {
        anchors.fill: parent
        radius: 28
        color: "#1C0607"
        opacity: 0.96
    }
    // 깜빡이는 빨간 테두리
    Rectangle {
        id: rim
        anchors.fill: parent
        radius: 28
        color: "transparent"
        border.color: Theme.red
        border.width: 10
        SequentialAnimation on border.width {
            running: ov.active
            loops: Animation.Infinite
            NumberAnimation { to: 26; duration: 280; easing.type: Easing.OutQuad }
            NumberAnimation { to: 10; duration: 280; easing.type: Easing.InQuad }
        }
    }

    // ---------- 가운데 문구 ----------
    Column {
        id: head
        anchors.horizontalCenter: parent.horizontalCenter
        y: parent.height * 0.07
        spacing: 2
        Row {
            anchors.horizontalCenter: parent.horizontalCenter
            spacing: 14
            Glyph { kind: "fault"; size: 54; color: Theme.red; anchors.verticalCenter: parent.verticalCenter }
            Text {
                anchors.verticalCenter: parent.verticalCenter
                text: "페달 오조작 감지"
                font.pixelSize: 34
                font.weight: Font.Bold
                color: "#FF7A7A"
            }
        }
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            text: "BRAKE NOW"
            font.pixelSize: 168
            font.weight: Font.Black
            font.letterSpacing: 4
            color: "white"
        }
        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            text: "브레이크를 즉시 밟으세요"
            font.pixelSize: 50
            font.weight: Font.Bold
            color: "#FFE1E1"
        }
    }

    // ---------- 페달 안내 ----------
    Row {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: head.bottom
        anchors.topMargin: 34
        spacing: 110

        // 가속 페달: 떼야 함
        Column {
            spacing: 14
            Rectangle {
                anchors.horizontalCenter: parent.horizontalCenter
                width: 96; height: 150; radius: 16
                color: ov.accReleased ? "#2D3036" : "#3A1012"
                border.color: ov.accReleased ? "#5D636C" : Theme.red
                border.width: 5
                Column {
                    anchors.centerIn: parent
                    spacing: 12
                    Repeater { model: 6; Rectangle { width: 56; height: 6; radius: 3.5; color: ov.accReleased ? "#5D636C" : "#FF8A8A" } }
                }
                // X 표시
                Text {
                    visible: !ov.accReleased
                    anchors.centerIn: parent
                    text: "✕"
                    font.pixelSize: 96
                    font.weight: Font.Black
                    color: Theme.red
                    opacity: 0.85
                }
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: ov.accReleased ? "가속 페달 해제됨" : "가속 페달에서 발 떼기"
                font.pixelSize: 28
                font.weight: Font.Bold
                color: ov.accReleased ? "#9FE3B5" : "#FF9C9C"
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "가속 " + Math.round(ov.acc) + " %"
                font.pixelSize: 24
                color: "#E8C9C9"
            }
        }

        // 브레이크: 밟아야 함
        Column {
            spacing: 14
            Rectangle {
                id: brakePedal
                anchors.horizontalCenter: parent.horizontalCenter
                width: 170; height: 150; radius: 20
                color: Theme.red
                border.color: "#FFB3B3"
                border.width: 5
                SequentialAnimation on scale {
                    running: ov.active
                    loops: Animation.Infinite
                    NumberAnimation { to: 1.08; duration: 280 }
                    NumberAnimation { to: 1.0; duration: 280 }
                }
                Column {
                    anchors.centerIn: parent
                    spacing: 12
                    Repeater { model: 5; Rectangle { width: 116; height: 8; radius: 4; color: "white" } }
                }
                // 밟은 만큼 아래에서 채움
                Rectangle {
                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: parent.width - 10
                    height: (parent.height - 10) * Theme.clamp01(ov.brk / 100)
                    radius: 16
                    color: Qt.rgba(1, 1, 1, 0.35)
                }
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "브레이크 밟기"
                font.pixelSize: 28
                font.weight: Font.Bold
                color: "white"
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "브레이크 " + Math.round(ov.brk) + " %"
                font.pixelSize: 24
                color: "#E8C9C9"
            }
        }
    }

    // ---------- 현재 상황 ----------
    Row {
        id: info
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 40
        spacing: 22
        Repeater {
            model: [
                ["속도", Theme.has(ov.s.speed_kmh) ? Math.round(ov.s.speed_kmh) + " km/h" : "--"],
                ["전방 거리", Theme.has(ov.s.distance_m) ? Theme.fmt(ov.s.distance_m, 1) + " m" : "--"],
                ["충돌 예상", Theme.has(ov.s.ttc_s) ? Theme.fmt(ov.s.ttc_s, 1) + " 초" : "--"]
            ]
            Rectangle {
                width: 250; height: 92; radius: 18
                color: Qt.rgba(1, 1, 1, 0.07)
                border.color: Qt.rgba(1, 0.4, 0.4, 0.45)
                Column {
                    anchors.centerIn: parent
                    spacing: 2
                    Text { anchors.horizontalCenter: parent.horizontalCenter; text: modelData[0]; font.pixelSize: 20; color: "#D9B5B5" }
                    Text { anchors.horizontalCenter: parent.horizontalCenter; text: modelData[1]; font.pixelSize: 36; font.weight: Font.Bold; color: "white" }
                }
            }
        }
    }
}
