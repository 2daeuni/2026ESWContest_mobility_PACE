import QtQuick
import "."

Card {
    id: sc
    property var s: ({})
    title: "PACE STATUS"

    readonly property var level: s.risk_level
    readonly property color tint: Theme.levelColor(level)
    readonly property real ringValue: Theme.has(s.total_risk) ? s.total_risk
                                     : (Theme.has(level) ? (level + 0.0) / 3.0 : 0)

    readonly property string chip: s.demo_drive === true ? "DEMO"
                                   : s.link_safety !== true ? "OFFLINE"
                                   : (s.failsafe_active ? "FAIL-SAFE"
                                   : (s.risk_available ? "ACTIVE" : "STANDBY"))
    readonly property color chipColor: chip === "ACTIVE" ? Theme.green
                                      : chip === "DEMO" ? Theme.orange
                                      : (chip === "FAIL-SAFE" ? Theme.red
                                      : (chip === "STANDBY" ? Theme.orange : Theme.gray))

    headerRight: Rectangle {
        width: chipText.width + 28; height: 34; radius: 8
        color: Qt.rgba(sc.chipColor.r, sc.chipColor.g, sc.chipColor.b, 0.16)
        y: -3
        Text {
            id: chipText
            anchors.centerIn: parent
            text: sc.chip
            font.pixelSize: 16
            font.weight: Font.Bold
            color: sc.chipColor
        }
    }

    Rectangle {   // 헤더 구분선
        width: parent.width; height: 1
        y: -8
        color: Theme.border
    }

    RiskRing {
        id: ring
        x: 2; y: 18
        width: 140; height: 140
        value: sc.ringValue
        level: sc.level
        levelText: Theme.has(sc.level) ? "L" + sc.level : "--"
        caption: Theme.has(sc.level) ? sc.s.risk_short : "N/A"
    }

    // Total Risk 박스
    Rectangle {
        id: totalBox
        anchors.left: ring.right
        anchors.leftMargin: 14
        y: 4
        width: 124; height: 170
        radius: 14
        color: Theme.card
        border.color: Theme.border

        Column {
            anchors.fill: parent
            anchors.margins: 16
            spacing: 10
            Text { text: "Total Risk"; font.pixelSize: 16; color: Theme.text }
            Text {
                text: Theme.fmt(sc.s.total_risk, 2)
                font.pixelSize: 34
                font.weight: Font.Bold
                color: Theme.has(sc.s.total_risk) ? sc.tint : Theme.gray
            }
            Item {
                width: parent.width; height: 32
                Rectangle {
                    width: parent.width; height: 10; radius: 5
                    gradient: Gradient {
                        orientation: Gradient.Horizontal
                        GradientStop { position: 0.0; color: "#FFD0A3" }
                        GradientStop { position: 0.5; color: Theme.orange }
                        GradientStop { position: 1.0; color: Theme.red }
                    }
                    opacity: Theme.has(sc.s.total_risk) ? 1 : 0.35
                }
                Rectangle {   // 현재값 마커
                    visible: Theme.has(sc.s.total_risk)
                    x: Theme.clamp01(sc.s.total_risk) * (parent.width - 4)
                    y: -3; width: 4; height: 16; radius: 2
                    color: Theme.text
                    Behavior on x { NumberAnimation { duration: 120 } }
                }
                Text { y: 16; text: "0"; font.pixelSize: 11; color: Theme.sub }
                Text { y: 16; x: parent.width / 2 - width / 2; text: "0.5"; font.pixelSize: 11; color: Theme.sub }
                Text { y: 16; x: parent.width - width; text: "1"; font.pixelSize: 11; color: Theme.sub }
            }
        }
    }

    // 세부 점수
    Column {
        anchors.left: totalBox.right
        anchors.leftMargin: 16
        anchors.right: parent.right
        y: 14
        spacing: 22
        Repeater {
            model: [
                { name: "BNS", value: Theme.fmt(sc.s.bns, 2), dot: Theme.red, hot: (sc.s.bns || 0) >= 0.5 },
                { name: "PMS", value: Theme.fmt(sc.s.pms, 2), dot: Theme.orange, hot: (sc.s.pms || 0) >= 0.5 },
                { name: "거리", value: Theme.has(sc.s.distance_m) ? Theme.fmt(sc.s.distance_m, 1) + "m" : "--", dot: Theme.gray, hot: false }
            ]
            Item {
                width: parent.width; height: 22
                Rectangle {
                    width: 10; height: 10; radius: 5
                    anchors.verticalCenter: parent.verticalCenter
                    color: modelData.dot
                }
                Text {
                    x: 18
                    anchors.verticalCenter: parent.verticalCenter
                    text: modelData.name
                    font.pixelSize: 16
                    color: Theme.text
                }
                Text {
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    text: modelData.value
                    font.pixelSize: 17
                    font.weight: Font.DemiBold
                    color: modelData.hot ? modelData.dot : Theme.text
                }
            }
        }
    }

    // 단계 표시 (L0~L3)
    Row {
        id: steps
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 4
        spacing: 8
        Repeater {
            model: [["L0", "정상"], ["L1", "주의"], ["L2", "경고"], ["L3", "위험"]]
            Rectangle {
                readonly property bool on: sc.level === index
                width: (steps.width - 24) / 4
                height: 34
                radius: 9
                color: on ? Theme.levelColor(index) : Theme.cardSoft
                border.color: on ? "transparent" : Theme.border
                Text {
                    anchors.centerIn: parent
                    text: modelData[0] + "  " + modelData[1]
                    font.pixelSize: 15
                    font.weight: parent.on ? Font.Bold : Font.Normal
                    color: parent.on ? "white" : Theme.sub
                }
            }
        }
    }
}
