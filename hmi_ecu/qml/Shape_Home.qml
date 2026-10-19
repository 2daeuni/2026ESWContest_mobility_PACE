import QtQuick
import QtQuick.Shapes
import "."

// 홈 아이콘
Shape {
    width: 26; height: 24
    preferredRendererType: Shape.CurveRenderer
    ShapePath {
        fillColor: Theme.sub
        strokeColor: Theme.sub
        strokeWidth: 2
        joinStyle: ShapePath.RoundJoin
        startX: 13; startY: 1
        PathLine { x: 25; y: 11 }
        PathLine { x: 22; y: 11 }
        PathLine { x: 22; y: 23 }
        PathLine { x: 16; y: 23 }
        PathLine { x: 16; y: 16 }
        PathLine { x: 10; y: 16 }
        PathLine { x: 10; y: 23 }
        PathLine { x: 4;  y: 23 }
        PathLine { x: 4;  y: 11 }
        PathLine { x: 1;  y: 11 }
        PathLine { x: 13; y: 1 }
    }
}
