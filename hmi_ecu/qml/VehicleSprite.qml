import QtQuick
import "."

// 차량 1대: CARLA 렌더링 이미지(assets/cars/*.png) 가 있으면 이미지, 없으면 기본 도형
// 위치 기준점 = 차량 뒷범퍼 바닥 중앙 (groundX, groundY)
Item {
    id: v
    property string sprite: ""
    property real groundX: 0
    property real groundY: 0
    property real carWidth: 100          // 화면상 차폭(px)
    property bool highlight: false
    property color highlightColor: Theme.red
    property color fallbackColor: "#C3C9D1"
    property bool brakeLights: false
    property bool walker: false

    readonly property bool hasImage: sprite !== "" && img.status === Image.Ready
    readonly property real imgAspect: img.implicitWidth > 0 ? img.implicitHeight / img.implicitWidth : 0.8
    // 이미지는 범퍼 아래 그림자 영역 약간 포함 → 바닥이 groundY 에 오도록 살짝 내림
    width: carWidth
    height: walker ? carWidth * 3.2 : carWidth * (hasImage ? imgAspect : 0.8)
    x: groundX - width / 2
    y: groundY - height + (hasImage ? height * 0.03 : 0)

    // 그림자
    Rectangle {
        visible: !v.walker
        width: v.width * 1.1
        height: Math.max(3, v.width * 0.14)
        x: -v.width * 0.05
        y: v.height - height * 0.6
        radius: height / 2
        color: "black"
        opacity: 0.18
    }

    Image {
        id: img
        anchors.fill: parent
        source: v.sprite
        visible: v.hasImage
        fillMode: Image.Stretch
        smooth: true
        mipmap: true
        asynchronous: true
        cache: true
    }

    VehicleRear {
        visible: !v.hasImage && !v.walker
        anchors.fill: parent
        bodyColor: v.fallbackColor
        brakeLights: v.brakeLights
    }

    // 보행자
    Column {
        visible: v.walker
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        spacing: 1
        Rectangle { anchors.horizontalCenter: parent.horizontalCenter; width: v.width * 0.9; height: width; radius: width / 2; color: "#5B6470" }
        Rectangle { anchors.horizontalCenter: parent.horizontalCenter; width: v.width * 1.3; height: v.width * 2.0; radius: v.width * 0.4; color: "#7A8491" }
    }

    // 강조 테두리 (앞차)
    Rectangle {
        visible: v.highlight
        anchors.fill: parent
        anchors.margins: -Math.max(3, v.width * 0.08)
        color: "transparent"
        border.width: 2
        border.color: v.highlightColor
        radius: 3
    }
}
