import QtQuick
import "."

// 하단 도크: PACE / 진단 탭 + DEMO 주행 토글 + (DEMO 중) 시나리오 컨트롤
Rectangle {
    id: dock
    property var s: ({})
    property int tab: 0
    signal tabRequested(int index)

    readonly property bool demoOn: s.demo_drive === true

    radius: 26
    color: Theme.card
    border.color: Theme.border

    Text {
        x: 36
        anchors.verticalCenter: parent.verticalCenter
        text: "PACE  ·  Intelligent Pedal Safety"
        font.pixelSize: 18
        font.weight: Font.DemiBold
        color: Theme.text
        visible: !dock.demoOn
    }

    // ---- DEMO 중: 왼쪽에 입력 안내 ----
    Column {
        x: 32
        anchors.verticalCenter: parent.verticalCenter
        visible: dock.demoOn
        spacing: 4
        Text {
            text: dock.s.demo_carla ? "DEMO 주행 · CARLA 연결" : "DEMO 주행"
            font.pixelSize: 18
            font.weight: Font.Bold
            color: Theme.orange
        }
        Text {
            text: dock.s.demo_input === "PEDAL" ? "입력: 실제 페달 (Pedal ECU)"
                  : dock.s.demo_carla ? "입력: CARLA 운전 창  W 가속 · S 브레이크 · A/D 핸들"
                  : "입력: 키보드  ↑ 가속 · ↓ 브레이크 · R 리셋"
            font.pixelSize: 14
            color: Theme.sub
        }
    }

    // ---- 가운데 버튼 ----
    Row {
        id: buttons
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.horizontalCenterOffset: -60
        anchors.verticalCenter: parent.verticalCenter
        spacing: 28

        Repeater {
            model: [
                { label: "PACE", glyph: "P", action: "tab0" },
                { label: "진단", glyph: "D", action: "tab1" },
                { label: "DEMO", glyph: "▶", action: "demo" }
            ]
            Column {
                spacing: 4
                readonly property bool active: modelData.action === "demo" ? dock.demoOn
                                               : (modelData.action === "tab0" ? dock.tab === 0 : dock.tab === 1)
                readonly property color accent: modelData.action === "demo" ? Theme.orange : Theme.blue
                Rectangle {
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: 56; height: 50; radius: 14
                    color: parent.active ? parent.accent : Theme.chip
                    Text {
                        anchors.centerIn: parent
                        text: modelData.action === "demo" && dock.demoOn ? "■" : modelData.glyph
                        font.pixelSize: modelData.action === "demo" ? 20 : 24
                        font.weight: Font.Black
                        color: parent.parent.active ? "white" : Theme.sub
                    }
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            if (modelData.action === "demo") {
                                hmi.toggleDemo()
                                dock.tabRequested(0)
                            } else {
                                dock.tabRequested(modelData.action === "tab0" ? 0 : 1)
                            }
                        }
                    }
                }
                Text {
                    anchors.horizontalCenter: parent.horizontalCenter
                    text: modelData.action === "demo" && dock.demoOn ? "DEMO 종료" : modelData.label
                    font.pixelSize: 15
                    color: parent.active ? parent.accent : Theme.sub
                }
            }
        }
    }

    // ---- 오른쪽: DEMO 컨트롤 or Risk 출처 ----
    Text {
        visible: !dock.demoOn
        anchors.right: parent.right
        anchors.rightMargin: 36
        anchors.verticalCenter: parent.verticalCenter
        text: "Risk: " + (dock.s.risk_source || "-")
        font.pixelSize: 16
        color: Theme.sub
    }

    Text {
        visible: dock.demoOn && dock.s.demo_carla === true
        anchors.right: parent.right
        anchors.rightMargin: 36
        anchors.verticalCenter: parent.verticalCenter
        text: "CARLA 실제 주행 값으로 위험 판정 중"
        font.pixelSize: 16
        color: Theme.sub
    }

    Row {
        visible: dock.demoOn && dock.s.demo_carla !== true
        anchors.right: parent.right
        anchors.rightMargin: 24
        anchors.verticalCenter: parent.verticalCenter
        spacing: 18

        Column {
            spacing: 5
            Text { text: "전방 차량 거리"; font.pixelSize: 13; color: Theme.sub }
            Row {
                spacing: 6
                Repeater {
                    model: [[1000, "1 km"], [300, "300 m"], [100, "100 m"]]
                    Chip {
                        text: modelData[1]
                        selected: dock.s.demo_start_distance === modelData[0]
                        onClicked: hmi.setDemoStartDistance(modelData[0])
                    }
                }
            }
        }
        Column {
            spacing: 5
            Text { text: "전방 차량 속도"; font.pixelSize: 13; color: Theme.sub }
            Row {
                spacing: 6
                Repeater {
                    model: [[0, "정지"], [30, "30"], [60, "60 km/h"]]
                    Chip {
                        text: modelData[1]
                        selected: dock.s.demo_lead_speed === modelData[0]
                        onClicked: hmi.setDemoLeadSpeed(modelData[0])
                    }
                }
            }
        }
        Rectangle {
            anchors.verticalCenter: parent.verticalCenter
            width: 96; height: 50; radius: 14
            color: Theme.text
            Text {
                anchors.centerIn: parent
                text: "RESET"
                font.pixelSize: 17
                font.weight: Font.Bold
                color: Theme.inverse
            }
            MouseArea { anchors.fill: parent; onClicked: hmi.demoReset() }
        }
    }
}
