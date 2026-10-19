import QtQuick
import QtQuick.Shapes
import "."

// 폰트 의존 없는 작은 아이콘. kind: warn | info | ok | fault | vib | limit | dot
Item {
    id: g
    property string kind: "info"
    property color color: Theme.blue
    property int size: 18
    width: size
    height: size

    // Steering wheel, using the existing vector glyph system.
    Rectangle {
        visible: g.kind === "steering"
        anchors.fill: parent
        anchors.margins: g.size * 0.08
        radius: width / 2
        color: "transparent"
        border.color: g.color
        border.width: g.size * 0.09
        Rectangle {
            anchors.centerIn: parent
            width: parent.width * 0.82
            height: g.size * 0.09
            radius: height / 2
            color: g.color
        }
        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            y: parent.height * 0.5
            width: g.size * 0.09
            height: parent.height * 0.43
            radius: width / 2
            color: g.color
        }
        Rectangle {
            anchors.centerIn: parent
            width: g.size * 0.25
            height: width
            radius: width / 2
            color: g.color
        }
    }

    // 경고 삼각형
    Shape {
        anchors.fill: parent
        visible: g.kind === "warn"
        preferredRendererType: Shape.CurveRenderer
        ShapePath {
            fillColor: g.color
            strokeColor: g.color
            strokeWidth: g.size * 0.08
            joinStyle: ShapePath.RoundJoin
            startX: g.size * 0.5; startY: g.size * 0.08
            PathLine { x: g.size * 0.96; y: g.size * 0.9 }
            PathLine { x: g.size * 0.04; y: g.size * 0.9 }
            PathLine { x: g.size * 0.5; y: g.size * 0.08 }
        }
    }
    Text {
        visible: g.kind === "warn"
        anchors.horizontalCenter: parent.horizontalCenter
        y: g.size * 0.22
        text: "!"
        color: "white"
        font.pixelSize: g.size * 0.62
        font.weight: Font.Black
    }

    // 원형 (info / ok / fault / limit)
    Rectangle {
        visible: g.kind === "info" || g.kind === "ok" || g.kind === "fault" || g.kind === "limit" || g.kind === "dot"
        anchors.fill: parent
        radius: width / 2
        color: g.color
        Text {
            anchors.centerIn: parent
            visible: g.kind !== "dot"
            text: g.kind === "info" ? "i" : (g.kind === "ok" ? "✓" : (g.kind === "limit" ? "↓" : "!"))
            color: "white"
            font.pixelSize: g.size * 0.62
            font.weight: Font.Black
        }
    }

    // 진동 (지그재그)
    Shape {
        anchors.fill: parent
        visible: g.kind === "vib"
        preferredRendererType: Shape.CurveRenderer
        ShapePath {
            fillColor: "transparent"
            strokeColor: g.color
            strokeWidth: Math.max(1.5, g.size * 0.1)
            joinStyle: ShapePath.RoundJoin
            capStyle: ShapePath.RoundCap
            startX: 0; startY: g.size * 0.5
            PathLine { x: g.size * 0.22; y: g.size * 0.5 }
            PathLine { x: g.size * 0.34; y: g.size * 0.15 }
            PathLine { x: g.size * 0.5;  y: g.size * 0.85 }
            PathLine { x: g.size * 0.66; y: g.size * 0.15 }
            PathLine { x: g.size * 0.78; y: g.size * 0.5 }
            PathLine { x: g.size;        y: g.size * 0.5 }
        }
    }
}
