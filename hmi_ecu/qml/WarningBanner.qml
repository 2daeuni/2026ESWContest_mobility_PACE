import QtQuick
import "."

Rectangle {
    id: b
    property var banner: ({ severity: "offline", title: "", subtitle: "" })
    property var ttc: null
    property bool big: false          // 운전자 화면용 큰 글씨
    readonly property string sev: banner && banner.severity ? banner.severity : "offline"
    readonly property color tone: Theme.severityColor(sev)
    readonly property bool alarming: sev === "danger" || sev === "warning" || sev === "fault"

    radius: 14
    color: Theme.severitySoft(sev)
    border.color: alarming ? Qt.rgba(tone.r, tone.g, tone.b, 0.35) : Theme.border

    // 위험 단계에서 깜빡임
    SequentialAnimation on opacity {
        running: b.sev === "danger"
        loops: Animation.Infinite
        NumberAnimation { to: 0.72; duration: 380 }
        NumberAnimation { to: 1.0; duration: 380 }
        onRunningChanged: if (!running) b.opacity = 1
    }

    Glyph {
        id: icon
        x: b.big ? 30 : 20
        anchors.verticalCenter: parent.verticalCenter
        size: b.big ? 68 : 40
        kind: b.alarming || b.sev === "caution" ? "warn" : (b.sev === "ok" ? "ok" : "info")
        color: b.sev === "offline" ? Theme.gray : b.tone
    }

    Column {
        anchors.left: icon.right
        anchors.leftMargin: 18
        anchors.right: ttcBox.left
        anchors.rightMargin: 10
        anchors.verticalCenter: parent.verticalCenter
        spacing: 4
        Text {
            width: parent.width
            text: b.banner ? b.banner.title : ""
            font.pixelSize: b.big ? 34 : 18
            font.weight: Font.Bold
            color: Theme.text
            elide: Text.ElideRight
        }
        Text {
            text: b.banner ? b.banner.subtitle : ""
            font.pixelSize: b.big ? 18 : 14
            color: Theme.sub
        }
    }

    Column {
        id: ttcBox
        anchors.right: parent.right
        anchors.rightMargin: 22
        anchors.verticalCenter: parent.verticalCenter
        spacing: 0
        visible: Theme.has(b.ttc)
        Text {
            anchors.right: parent.right
            text: "TTC"
            font.pixelSize: b.big ? 18 : 14
            color: Theme.sub
        }
        Row {
            anchors.right: parent.right
            spacing: 3
            Text {
                id: ttcNum
                text: Theme.fmt(b.ttc, 1)
                font.pixelSize: b.big ? 52 : 28
                font.weight: Font.DemiBold
                color: b.alarming ? b.tone : Theme.text
            }
            Text {
                text: "s"
                font.pixelSize: b.big ? 28 : 18
                color: ttcNum.color
                anchors.baseline: ttcNum.baseline
            }
        }
    }
}
