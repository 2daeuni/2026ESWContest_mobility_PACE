import QtQuick
import "."

// 왼쪽 클러스터 (테슬라 화면 2번 구역 구성)
//   위: P R N D / 큰 속도 / 상태
//   가운데: 3D 주행 화면 (자차 + CARLA 주변 차량 / DEMO 전방 차량)
//   아래: 페달 (가속 / 브레이크 / 변화율 / 입력 출처)
Rectangle {
    id: cl
    property var s: ({})
    property var world: null
    property string mode: "LIVE"

    color: Theme.dark ? "#16191E" : "#E9EBEE"       // 3D 화면 바탕과 같은 색 → 경계 없이 이어짐
    radius: 28
    border.color: Theme.border
    clip: true

    readonly property string gear: Theme.has(s.gear) ? s.gear : (s.demo_drive || (Theme.has(s.speed_kmh) && s.speed_kmh > 0.5) ? "D" : "")
    readonly property string status: s.demo_drive ? "READY" : (s.link_safety !== true ? "OFFLINE" : (s.failsafe_active ? "FAIL-SAFE" : "READY"))
    readonly property color statusColor: status === "READY" ? Theme.green : (status === "FAIL-SAFE" ? Theme.red : Theme.gray)

    // ---------------- 상단 ----------------
    Item {
        id: top
        width: parent.width
        height: 168

        // 기어
        Row {
            x: 30; y: 30
            spacing: 10
            Repeater {
                model: ["P", "R", "N", "D"]
                Text {
                    text: modelData
                    font.pixelSize: 21
                    font.weight: cl.gear === modelData ? Font.Black : Font.Normal
                    color: cl.gear === modelData ? Theme.text : Theme.faint
                }
            }
        }

        // 속도
        Text {
            id: speedText
            anchors.horizontalCenter: parent.horizontalCenter
            y: 2
            text: Theme.has(cl.s.speed_kmh) ? Math.round(cl.s.speed_kmh) : "--"
            font.pixelSize: 92
            font.weight: Font.Medium
            color: Theme.has(cl.s.speed_kmh) ? Theme.text : Theme.faint
        }
        Text {
            id: unitText
            anchors.horizontalCenter: parent.horizontalCenter
            y: 110
            text: "KM/H"
            font.pixelSize: 16
            font.letterSpacing: 1
            color: Theme.text
        }

        // 상태 / 모드
        Column {
            anchors.right: parent.right
            anchors.rightMargin: 28
            y: 26
            spacing: 10
            Rectangle {
                anchors.right: parent.right
                readonly property string label: cl.s.demo_carla ? "DEMO · CARLA" : (cl.s.demo_drive ? "DEMO" : cl.mode)
                width: modeText.width + 18; height: 24; radius: 12
                color: label !== "LIVE" ? Qt.rgba(1, 0.54, 0.12, 0.16) : Qt.rgba(0.18, 0.48, 0.92, 0.14)
                Text {
                    id: modeText
                    anchors.centerIn: parent
                    text: parent.label
                    font.pixelSize: 12
                    font.weight: Font.Bold
                    color: parent.label !== "LIVE" ? Theme.orange : Theme.blue
                }
            }
            Text {
                anchors.right: parent.right
                text: cl.status
                font.pixelSize: 17
                font.weight: Font.Bold
                color: cl.statusColor
            }
            Glyph {
                anchors.right: parent.right
                anchors.rightMargin: 12
                kind: "steering"
                size: 26
                visible: cl.s.lane_assist_enabled === true
                color: cl.s.lane_assist_active === true ? Theme.green : Theme.gray
            }
        }

        Text {
            anchors.horizontalCenter: parent.horizontalCenter
            y: 132
            text: "속도 출처 " + (cl.s.speed_source || "-")
            font.pixelSize: 12
            color: Theme.sub
        }

        Rectangle {
            anchors.bottom: parent.bottom
            x: 24; width: parent.width - 48; height: 1
            color: Theme.dark ? "#2A3038" : "#D9DCE1"
        }
    }

    // ---------------- 3D 주행 화면 ----------------
    DriveVis3D {
        id: vis
        y: top.height
        width: parent.width
        height: pedals.y - y - 8
        world: cl.world
    }

    // ---------------- 페달 ----------------
    Rectangle {
        id: pedals
        x: 16
        width: parent.width - 32
        height: 176
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 16
        radius: 20
        color: Theme.card
        border.color: cl.s.brake_now ? Theme.red : Theme.border
        border.width: cl.s.brake_now ? 2 : 1

        Column {
            x: 22; y: 18
            width: parent.width - 44
            spacing: 14

            Repeater {
                model: [
                    { "name": "가속 페달", "v": cl.s.accelerator, "rate": cl.s.accelerator_rate, "c": cl.s.brake_now ? Theme.red : Theme.green },
                    { "name": "브레이크", "v": cl.s.brake, "rate": cl.s.brake_rate, "c": Theme.blue }
                ]
                Item {
                    width: parent.width
                    height: 40
                    Text { text: modelData.name; font.pixelSize: 14; color: Theme.sub }
                    Text {
                        anchors.right: parent.right
                        text: (Theme.has(modelData.rate) ? Theme.fmt(modelData.rate, 0) + " %/s   " : "")
                        font.pixelSize: 12
                        color: Theme.faint
                        anchors.rightMargin: 64
                    }
                    Text {
                        anchors.right: parent.right
                        text: Theme.fmt(modelData.v, 0) + " %"
                        font.pixelSize: 16
                        font.weight: Font.DemiBold
                        color: Theme.text
                    }
                    Rectangle {
                        y: 26; width: parent.width; height: 9; radius: 4.5
                        color: Theme.track
                        Rectangle {
                            width: parent.width * Theme.clamp01((modelData.v || 0) / 100)
                            height: parent.height; radius: parent.radius
                            color: modelData.c
                            Behavior on width { NumberAnimation { duration: 80 } }
                        }
                    }
                }
            }

            Row {
                spacing: 18
                Text {
                    text: "입력  " + (cl.s.demo_drive ? (cl.s.demo_input === "PEDAL" ? "PEDAL (CAN)" : "KEYBOARD") : (cl.s.pedal_source || "-"))
                    font.pixelSize: 12; color: Theme.sub
                }
                Text { text: "위험도 출처  " + (cl.s.risk_source || "-"); font.pixelSize: 12; color: Theme.sub }
            }
        }
    }
}
