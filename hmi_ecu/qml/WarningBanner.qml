import QtQuick
import "."

Rectangle {
    id: b
    property var banner: ({ severity: "offline", title: "", subtitle: "" })
    property var ttc: null
    property bool big: false          // 운전자 화면용 큰 글씨
    property bool prominent: false
    readonly property string sev: banner && banner.severity ? banner.severity : "offline"
    readonly property color tone: Theme.severityColor(sev)
    readonly property bool alarming: sev === "danger" || sev === "warning" || sev === "fault"

    radius: 14
    color: prominent ? "#171B24" : Theme.severitySoft(sev)
    border.width: prominent ? 6 : 1
    border.color: prominent ? tone : (alarming ? Qt.rgba(tone.r, tone.g, tone.b, 0.35) : Theme.border)

    // 위험 단계에서 깜빡임
    SequentialAnimation on opacity {
        running: b.sev === "danger" && !b.prominent
        loops: Animation.Infinite
        NumberAnimation { to: 0.72; duration: 380 }
        NumberAnimation { to: 1.0; duration: 380 }
        onRunningChanged: if (!running) b.opacity = 1
    }

    Glyph {
        id: icon
        visible: !b.prominent
        x: b.big ? 30 : 20
        anchors.verticalCenter: parent.verticalCenter
        size: b.big ? 68 : 40
        kind: b.alarming || b.sev === "caution" ? "warn" : (b.sev === "ok" ? "ok" : "info")
        color: b.sev === "offline" ? Theme.gray : b.tone
    }

    Column {
        anchors.left: b.prominent ? parent.left : icon.right
        anchors.leftMargin: b.prominent ? 24 : 18
        anchors.right: b.prominent ? parent.right : ttcBox.left
        anchors.rightMargin: b.prominent ? 24 : 10
        anchors.verticalCenter: parent.verticalCenter
        spacing: 4
        Text {
            width: parent.width
            height: b.prominent ? b.height * 0.62 : implicitHeight
            text: b.banner ? b.banner.title : ""
            font.pixelSize: b.prominent ? Math.min(92, b.height * 0.52) : (b.big ? 34 : 18)
            fontSizeMode: b.prominent ? Text.Fit : Text.FixedSize
            minimumPixelSize: b.prominent ? 48 : 18
            font.weight: Font.Bold
            horizontalAlignment: b.prominent ? Text.AlignHCenter : Text.AlignLeft
            verticalAlignment: Text.AlignVCenter
            color: b.prominent ? (b.sev === "danger" ? "#FFFFFF" : b.tone) : Theme.text
            elide: Text.ElideRight
        }
        Text {
            width: parent.width
            text: b.banner ? b.banner.subtitle : ""
            font.pixelSize: b.prominent ? 23 : (b.big ? 18 : 14)
            horizontalAlignment: b.prominent ? Text.AlignHCenter : Text.AlignLeft
            color: b.prominent ? "#F2F4F8" : Theme.sub
        }
    }

    Column {
        id: ttcBox
        anchors.right: parent.right
        anchors.rightMargin: 22
        anchors.verticalCenter: parent.verticalCenter
        spacing: 0
        visible: !b.prominent && Theme.has(b.ttc)
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
