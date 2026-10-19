import QtQuick
import "."

// 통합 디버깅용: 메시지별 수신 주기/나이/Raw, 소스 상태, 이벤트 전체, 병합 상태 값
Item {
    id: page
    property var s: ({})
    property var diag: []
    property var events: []
    property var links: ({})
    property var perf: ({})

    Card {
        id: busCard
        x: 0; y: 0
        width: page.width; height: 350
        title: "CAN / UDP 수신 현황"
        subtitle: "HMI 는 수신 전용 (버스 송신 없음)"

        Row {
            id: head
            spacing: 0
            Repeater {
                model: [["종류", 60], ["ID", 76], ["메시지", 190], ["주기", 90], ["Age", 90], ["Alive", 70], ["Raw Data", 320], ["상태", 90]]
                Text { width: modelData[1]; text: modelData[0]; font.pixelSize: 13; color: Theme.sub; font.weight: Font.DemiBold }
            }
        }
        Column {
            y: 24
            spacing: 2
            Repeater {
                model: page.diag
                Row {
                    height: 26
                    Text { width: 60; text: modelData.kind; font.pixelSize: 15; color: Theme.sub }
                    Text { width: 76; text: modelData.id; font.pixelSize: 15; font.family: "monospace"; color: Theme.text }
                    Text { width: 190; text: modelData.name; font.pixelSize: 15; color: Theme.text }
                    Text { width: 90; text: modelData.rate + " Hz"; font.pixelSize: 15; color: Theme.text }
                    Text { width: 90; text: Theme.has(modelData.age) ? modelData.age + " ms" : "-"; font.pixelSize: 15; color: modelData.ok ? Theme.text : Theme.red }
                    Text { width: 70; text: Theme.has(modelData.alive) ? modelData.alive : "-"; font.pixelSize: 15; color: Theme.text }
                    Text { width: 320; text: modelData.raw; font.pixelSize: 15; font.family: "monospace"; color: Theme.text; elide: Text.ElideRight }
                    Row {
                        spacing: 6
                        Rectangle { width: 10; height: 10; radius: 5; anchors.verticalCenter: parent.verticalCenter; color: modelData.ok ? Theme.green : (modelData.count > 0 ? Theme.red : Theme.gray) }
                        Text { text: modelData.ok ? "OK" : (modelData.count > 0 ? "TIMEOUT" : "없음"); font.pixelSize: 14; color: Theme.sub }
                    }
                }
            }
        }
        Text {
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 22
            text: "성능  ·  화면 " + Theme.fmt(page.perf.fps, 0) + " fps  ·  HMI CPU " + Theme.fmt(page.perf.cpu, 1)
                  + " %  ·  상태 계산 " + Theme.fmt(page.perf.tick_ms, 2) + " ms  ·  페달 입력 → 화면 상태 "
                  + Theme.fmt(page.perf.input_ms, 1) + " ms (+ 1프레임 표시)  ·  경고음 " + (page.s.sound_backend || "-")
            font.pixelSize: 14
            font.weight: Font.DemiBold
            color: Theme.text
        }
        Text {
            anchors.bottom: parent.bottom
            text: "CAN: " + (page.links.can ? page.links.can.text : "-")
                  + "     |     " + (page.links.udp ? page.links.udp.text : "-")
                  + (page.links.dropped ? "     |     dropped " + page.links.dropped : "")
            font.pixelSize: 14
            color: Theme.sub
        }
    }

    Card {
        x: 0; y: busCard.height + 10
        width: (page.width - 10) * 0.48
        height: page.height - y
        title: "EVENT LOG"
        clip: true
        ListView {
            anchors.fill: parent
            clip: true
            model: page.events
            spacing: 2
            delegate: Row {
                spacing: 12
                height: 24
                Text { text: modelData.time; font.pixelSize: 14; color: Theme.sub }
                Rectangle { width: 9; height: 9; radius: 5; anchors.verticalCenter: parent.verticalCenter; color: Theme.severityColor(modelData.severity) }
                Text { text: modelData.text; font.pixelSize: 14; color: Theme.text }
            }
        }
    }

    Card {
        x: (page.width - 10) * 0.48 + 10; y: busCard.height + 10
        width: page.width - x
        height: page.height - y
        title: "병합 상태 (state)"
        Flow {
            anchors.fill: parent
            spacing: 4
            flow: Flow.TopToBottom
            Repeater {
                model: [
                    "risk_level", "risk_available", "risk_source", "total_risk", "bns", "pms",
                    "distance_m", "ttc_s", "speed_kmh", "gear",
                    "accelerator", "brake", "pedal_source", "pedal_sensor_status",
                    "failsafe_active", "failsafe_reason",
                    "haptic_cmd_on", "haptic_cmd_intensity", "haptic_cmd_frequency", "haptic_ready", "haptic_fault",
                    "motor_mode", "motor_limit", "brake_now"
                ]
                Row {
                    width: 250
                    height: 18
                    Text { width: 172; text: modelData; font.pixelSize: 12; color: Theme.sub; font.family: "monospace" }
                    Text {
                        readonly property var v: page.s[modelData]
                        text: v === undefined || v === null ? "null"
                              : (typeof v === "number" ? (Number.isInteger(v) ? v : v.toFixed(2)) : String(v))
                        font.pixelSize: 13
                        font.family: "monospace"
                        color: Theme.text
                    }
                }
            }
        }
    }
}
