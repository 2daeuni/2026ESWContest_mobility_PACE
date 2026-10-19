import QtQuick
import "."

// 단순화한 차량 뒷모습 (이미지 파일 없이 사각형으로만 구성 → Pi 에서 가볍게)
Item {
    id: car
    property color bodyColor: "#C9CED6"
    property color glassColor: "#3B4048"
    property bool brakeLights: false
    width: 120
    height: width * 0.8

    // 그림자
    Rectangle {
        width: car.width * 1.08
        height: car.height * 0.16
        x: -car.width * 0.04
        y: car.height * 0.9
        radius: height / 2
        color: "#000000"
        opacity: 0.12
    }
    // 바퀴
    Rectangle { x: car.width * 0.06; y: car.height * 0.74; width: car.width * 0.17; height: car.height * 0.24; radius: 3; color: "#1E2126" }
    Rectangle { x: car.width * 0.77; y: car.height * 0.74; width: car.width * 0.17; height: car.height * 0.24; radius: 3; color: "#1E2126" }
    // 지붕/유리
    Rectangle {
        x: car.width * 0.15; y: 0
        width: car.width * 0.7; height: car.height * 0.42
        radius: car.width * 0.08
        color: car.bodyColor
        Rectangle {
            anchors.fill: parent
            anchors.margins: car.width * 0.035
            anchors.bottomMargin: car.width * 0.02
            radius: car.width * 0.05
            color: car.glassColor
            gradient: Gradient {
                GradientStop { position: 0.0; color: Qt.lighter(car.glassColor, 1.5) }
                GradientStop { position: 1.0; color: car.glassColor }
            }
        }
    }
    // 차체
    Rectangle {
        x: 0; y: car.height * 0.36
        width: car.width; height: car.height * 0.46
        radius: car.width * 0.09
        color: car.bodyColor
        gradient: Gradient {
            GradientStop { position: 0.0; color: Qt.lighter(car.bodyColor, 1.08) }
            GradientStop { position: 1.0; color: Qt.darker(car.bodyColor, 1.12) }
        }
        // 테일램프
        Rectangle {
            x: parent.width * 0.05; y: parent.height * 0.18
            width: parent.width * 0.3; height: parent.height * 0.2
            radius: height / 2
            color: car.brakeLights ? "#FF2A2A" : "#B3262B"
        }
        Rectangle {
            x: parent.width * 0.65; y: parent.height * 0.18
            width: parent.width * 0.3; height: parent.height * 0.2
            radius: height / 2
            color: car.brakeLights ? "#FF2A2A" : "#B3262B"
        }
        // 번호판
        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            y: parent.height * 0.48
            width: parent.width * 0.26; height: parent.height * 0.2
            radius: 2
            color: "#F4F5F7"
        }
        // 범퍼
        Rectangle {
            anchors.bottom: parent.bottom
            width: parent.width; height: parent.height * 0.18
            radius: car.width * 0.06
            color: Qt.darker(car.bodyColor, 1.35)
        }
    }
}
