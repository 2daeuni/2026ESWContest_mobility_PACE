import QtQuick
import QtQuick.Shapes
import "."

// 위에서 본 자차 + 전방 감지 영역 (ToF/CARLA 거리)
Item {
    id: root
    property var distance: null
    property var level: null
    property real maxRange: 40          // m, 감지 영역 최대 표시 거리
    readonly property color tint: Theme.levelColor(level)

    width: 300
    height: 380

    readonly property real carW: 116
    readonly property real carH: 220
    readonly property real carX: (width - carW) / 2
    readonly property real carY: height - carH - 10
    readonly property real coneLen: carY - 8
    readonly property real hitRatio: Theme.has(distance) ? Math.max(0.05, Math.min(1, distance / maxRange)) : 1

    // 바닥 원형 플랫폼
    Rectangle {
        width: root.width * 1.05
        height: 70
        x: (root.width - width) / 2
        y: root.height - 60
        radius: height / 2
        color: Theme.platform
        border.color: Theme.platformEdge
    }

    // 감지 콘 (전체 범위)
    Shape {
        anchors.fill: parent
        preferredRendererType: Shape.CurveRenderer
        ShapePath {
            strokeColor: "transparent"
            fillGradient: LinearGradient {
                x1: 0; y1: root.carY; x2: 0; y2: root.carY - root.coneLen
                GradientStop { position: 0; color: Qt.rgba(0.18, 0.48, 0.92, 0.22) }
                GradientStop { position: 1; color: Qt.rgba(0.18, 0.48, 0.92, 0.02) }
            }
            startX: root.width / 2 - 26; startY: root.carY + 4
            PathLine { x: root.width / 2 - 92; y: root.carY - root.coneLen }
            PathLine { x: root.width / 2 + 92; y: root.carY - root.coneLen }
            PathLine { x: root.width / 2 + 26; y: root.carY + 4 }
        }
    }

    // 장애물 위치 마커
    Rectangle {
        visible: Theme.has(root.distance)
        width: 26 + 66 * root.hitRatio
        height: 6
        radius: 3
        x: (root.width - width) / 2
        y: root.carY - root.coneLen * root.hitRatio
        color: root.tint
        Behavior on y { NumberAnimation { duration: 90 } }
    }
    Text {
        visible: Theme.has(root.distance)
        anchors.horizontalCenter: parent.horizontalCenter
        y: root.carY - root.coneLen * root.hitRatio - 26
        text: Theme.fmt(root.distance, 1) + " m"
        color: root.tint
        font.pixelSize: 16
        font.weight: Font.Bold
    }

    // 차체
    Item {
        x: root.carX; y: root.carY
        width: root.carW; height: root.carH

        // 바퀴
        Repeater {
            model: [[-7, 34], [root.carW - 9, 34], [-7, root.carH - 74], [root.carW - 9, root.carH - 74]]
            Rectangle { x: modelData[0]; y: modelData[1]; width: 16; height: 44; radius: 5; color: "#262A30" }
        }
        Rectangle {
            anchors.fill: parent
            radius: 38
            gradient: Gradient {
                orientation: Gradient.Horizontal
                GradientStop { position: 0.0; color: "#B9BFC8" }
                GradientStop { position: 0.5; color: "#E9ECF0" }
                GradientStop { position: 1.0; color: "#B9BFC8" }
            }
            border.color: "#A9B0BA"
        }
        // 앞유리
        Rectangle { x: 14; y: 52; width: root.carW - 28; height: 44; radius: 12; color: "#3A4049" }
        // 지붕
        Rectangle { x: 18; y: 100; width: root.carW - 36; height: 64; radius: 8; color: "#CDD2D9" }
        // 뒷유리
        Rectangle { x: 18; y: 166; width: root.carW - 36; height: 26; radius: 9; color: "#3A4049" }
        // 헤드램프
        Rectangle { x: 14; y: 8; width: 26; height: 8; radius: 4; color: "#F7F9FC"; border.color: "#AAB2BC" }
        Rectangle { x: root.carW - 40; y: 8; width: 26; height: 8; radius: 4; color: "#F7F9FC"; border.color: "#AAB2BC" }
        // 전방 센서
        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            y: -3; width: 18; height: 8; radius: 3
            color: Theme.has(root.distance) ? Theme.blue : Theme.gray
        }
    }
}
