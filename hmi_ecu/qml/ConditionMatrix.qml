import QtQuick
import "."

// 위험 판정 근거: 단계별 (거리, TTC, 페달) 3조건 중 2개 이상이면 그 단계 후보
// conditions = {"1":[bool,bool,bool], "2":[...], "3":[...]} 또는 null
Item {
    id: m
    property var conditions: null
    property var thresholds: null      // {"1":["< 80 m","< 6 s","≥ 10 %"], ...} (DEMO 만)
    property var level: null
    property int rowH: 24
    property int labelW: 40

    readonly property bool hasData: conditions !== null && conditions !== undefined
    readonly property real colW: (width - labelW) / 3

    height: 18 + rowH * 3

    // 헤더
    Repeater {
        model: ["거리", "TTC", "페달"]
        Text {
            visible: m.hasData
            x: m.labelW + index * m.colW
            width: m.colW
            horizontalAlignment: Text.AlignHCenter
            text: modelData
            font.pixelSize: 12
            color: Theme.sub
        }
    }

    Repeater {
        model: 3
        Rectangle {
            readonly property string key: String(index + 1)
            readonly property var row: m.hasData ? m.conditions[key] : null
            readonly property int count: row ? (row[0] ? 1 : 0) + (row[1] ? 1 : 0) + (row[2] ? 1 : 0) : 0
            readonly property bool met: count >= 2
            readonly property color tone: Theme.levelColor(index + 1)
            y: 18 + index * m.rowH
            width: m.width
            height: m.rowH - 2
            radius: 6
            color: met ? Qt.rgba(tone.r, tone.g, tone.b, 0.16) : "transparent"
            border.color: m.level === index + 1 ? tone : "transparent"
            border.width: 2

            Text {
                x: 8
                anchors.verticalCenter: parent.verticalCenter
                text: "L" + (index + 1)
                font.pixelSize: 13
                font.weight: parent.met ? Font.Bold : Font.Normal
                color: parent.met ? parent.tone : Theme.sub
            }

            Repeater {
                model: 3
                Row {
                    readonly property bool on: parent.row ? parent.row[index] === true : false
                    readonly property string th: m.thresholds && m.thresholds[parent.key] ? m.thresholds[parent.key][index] : ""
                    x: m.labelW + index * m.colW + (m.colW - width) / 2
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 5
                    Rectangle {
                        anchors.verticalCenter: parent.verticalCenter
                        width: 11; height: 11; radius: 6
                        color: parent.on ? parent.parent.tone : "transparent"
                        border.color: parent.on ? parent.parent.tone : Theme.faint
                        border.width: 2
                        opacity: m.hasData ? 1 : 0.4
                    }
                    Text {
                        visible: parent.th !== ""
                        anchors.verticalCenter: parent.verticalCenter
                        text: parent.th
                        font.pixelSize: 12
                        color: parent.on ? Theme.text : Theme.sub
                    }
                }
            }
        }
    }

    Text {
        visible: !m.hasData
        anchors.right: parent.right
        y: 0
        text: "근거 데이터 없음"
        font.pixelSize: 11
        color: Theme.faint
    }
}
