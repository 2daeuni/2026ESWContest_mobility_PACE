import QtQuick
import "."

// 반응 시간 측정이 끝나면 6초간 결과 표시
Rectangle {
    id: toast
    property var trial: null
    visible: opacity > 0
    opacity: 0
    width: content.width + 40
    height: 56
    radius: 28
    color: Theme.text
    Behavior on opacity { NumberAnimation { duration: 200 } }

    function show(t) {
        trial = t
        opacity = 0.96
        hideTimer.restart()
    }
    Timer { id: hideTimer; interval: 6000; onTriggered: toast.opacity = 0 }

    readonly property color oc: !trial ? Theme.gray
                               : trial.outcome === "충돌" ? Theme.red
                               : trial.outcome === "회피" ? Theme.green : Theme.orange

    Row {
        id: content
        anchors.centerIn: parent
        spacing: 18
        Text {
            anchors.verticalCenter: parent.verticalCenter
            text: "반응 시간"
            font.pixelSize: 15
            color: Theme.inverse
            opacity: 0.7
        }
        Text {
            anchors.verticalCenter: parent.verticalCenter
            text: toast.trial ? "가속 해제 " + Theme.fmt(toast.trial.release_s, 2) + " s" : ""
            font.pixelSize: 19
            font.weight: Font.Bold
            color: Theme.inverse
        }
        Text {
            anchors.verticalCenter: parent.verticalCenter
            text: toast.trial ? "브레이크 " + Theme.fmt(toast.trial.brake_s, 2) + " s" : ""
            font.pixelSize: 19
            font.weight: Font.Bold
            color: Theme.inverse
        }
        Rectangle {
            anchors.verticalCenter: parent.verticalCenter
            width: outText.width + 20; height: 30; radius: 15
            color: toast.oc
            Text { id: outText; anchors.centerIn: parent; text: toast.trial ? toast.trial.outcome : ""; font.pixelSize: 15; font.weight: Font.Bold; color: "white" }
        }
    }
}
