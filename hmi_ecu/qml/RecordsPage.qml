import QtQuick
import "."

// 반응 시간 기록 (행동교정 효과 검증용)
Item {
    id: page
    property var trials: []
    property var summary: []
    property bool measuring: false
    property bool muted: false

    Card {
        id: sumCard
        x: 0; y: 0
        width: page.width; height: 214
        title: "반응 시간 요약"
        subtitle: "경고(L2 이상) 시작 → 가속 페달 해제 / 브레이크"

        headerRight: Row {
            spacing: 10
            Rectangle {
                visible: page.measuring
                width: measText.width + 24; height: 30; radius: 15
                color: Theme.redSoft
                Text { id: measText; anchors.centerIn: parent; text: "● 측정 중"; font.pixelSize: 14; font.weight: Font.Bold; color: Theme.red }
            }
            Chip { text: "기록 지우기"; onClicked: hmi.clearTrials() }
        }

        Row {
            id: sumRow
            spacing: 16
            Repeater {
                model: page.summary
                Rectangle {
                    width: 330; height: 112; radius: 14
                    color: Theme.cardSoft
                    Column {
                        x: 18; y: 14
                        spacing: 8
                        Text {
                            text: "경고음 " + modelData.sound + "  ·  " + modelData.n + "회"
                            font.pixelSize: 15; font.weight: Font.Bold
                            color: modelData.sound === "ON" ? Theme.blue : Theme.sub
                        }
                        Row {
                            spacing: 28
                            Column {
                                Text { text: "가속 해제 평균"; font.pixelSize: 12; color: Theme.sub }
                                Text { text: Theme.fmt(modelData.release_avg, 2) + " s"; font.pixelSize: 26; font.weight: Font.DemiBold; color: Theme.text }
                            }
                            Column {
                                Text { text: "브레이크 평균"; font.pixelSize: 12; color: Theme.sub }
                                Text { text: Theme.fmt(modelData.brake_avg, 2) + " s"; font.pixelSize: 26; font.weight: Font.DemiBold; color: Theme.text }
                            }
                            Column {
                                Text { text: "충돌"; font.pixelSize: 12; color: Theme.sub }
                                Text { text: modelData.collisions + "회"; font.pixelSize: 26; font.weight: Font.DemiBold; color: modelData.collisions > 0 ? Theme.red : Theme.text }
                            }
                        }
                    }
                }
            }
            Column {
                width: 300
                spacing: 6
                Text {
                    width: parent.width
                    wrapMode: Text.WordWrap
                    text: page.summary.length === 0 ? "아직 측정 기록 없음. DEMO 주행이나 실차에서 L2 이상 경고가 뜨면 자동으로 측정됨." :
                          "같은 시나리오를 경고음 ON / OFF 로 번갈아 반복하면 조건별 평균을 비교할 수 있음."
                    font.pixelSize: 13
                    color: Theme.sub
                }
                Text {
                    text: "현재 경고음: " + (page.muted ? "OFF" : "ON") + " (헤더에서 전환)"
                    font.pixelSize: 13
                    color: Theme.text
                }
                Text {
                    text: "CSV 저장: hmi_ecu/logs/reaction_log.csv"
                    font.pixelSize: 12
                    font.family: "monospace"
                    color: Theme.sub
                }
            }
        }
    }

    Card {
        id: recCard
        x: 0; y: sumCard.height + 10
        width: page.width; height: page.height - y
        title: "측정 기록"
        subtitle: "최신순"

        readonly property var cols: [
            ["시각", 84], ["모드", 62], ["입력", 92], ["소리", 50], ["최고", 50], ["경고 시 속도", 104],
            ["경고 시 거리", 104], ["가속 해제", 90], ["브레이크", 90], ["최소 거리", 90], ["결과", 80]
        ]

        Row {
            id: head
            Repeater {
                model: recCard.cols
                Text { width: modelData[1]; text: modelData[0]; font.pixelSize: 13; font.weight: Font.DemiBold; color: Theme.sub }
            }
        }
        ListView {
            anchors.fill: parent
            anchors.topMargin: 26
            clip: true
            model: page.trials
            spacing: 2
            delegate: Row {
                height: 26
                readonly property color oc: modelData.outcome === "충돌" ? Theme.red : (modelData.outcome === "회피" ? Theme.green : Theme.orange)
                Text { width: 84; text: modelData.time; font.pixelSize: 14; color: Theme.sub }
                Text { width: 62; text: modelData.mode; font.pixelSize: 14; color: Theme.text }
                Text { width: 92; text: modelData.input; font.pixelSize: 14; color: Theme.text; elide: Text.ElideRight }
                Text { width: 50; text: modelData.sound; font.pixelSize: 14; color: modelData.sound === "ON" ? Theme.blue : Theme.sub }
                Text { width: 50; text: "L" + modelData.max_level; font.pixelSize: 14; color: Theme.levelColor(modelData.max_level) }
                Text { width: 104; text: Theme.fmt(modelData.speed_at_warning_kmh, 0) + " km/h"; font.pixelSize: 14; color: Theme.text }
                Text { width: 104; text: Theme.fmt(modelData.distance_at_warning_m, 1) + " m"; font.pixelSize: 14; color: Theme.text }
                Text { width: 90; text: Theme.fmt(modelData.release_s, 2) + " s"; font.pixelSize: 14; font.weight: Font.DemiBold; color: Theme.text }
                Text { width: 90; text: Theme.fmt(modelData.brake_s, 2) + " s"; font.pixelSize: 14; font.weight: Font.DemiBold; color: Theme.text }
                Text { width: 90; text: Theme.fmt(modelData.min_distance_m, 1) + " m"; font.pixelSize: 14; color: Theme.text }
                Text { width: 80; text: modelData.outcome; font.pixelSize: 14; font.weight: Font.Bold; color: parent.oc }
            }
        }
    }
}
