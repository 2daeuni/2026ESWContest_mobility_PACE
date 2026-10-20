import QtQuick
import QtQuick3D
import "."

// 운전자 화면: 운전 중 한눈에 볼 것만 크게 (경고 문구, 도로 장면, 위험 단계)
// CAN ID / 점수 / 로그 같은 엔지니어 정보는 "상세 화면"에서
Item {
    id: page
    property var s: ({})
    property var world: null

    WarningBanner {
        id: banner
        x: 0; y: 0
        width: page.width
        height: 128
        big: true
        banner: page.s.banner
        ttc: page.s.ttc_s
    }

    Rectangle {
        id: sceneCard
        y: banner.height + 12
        width: page.width
        height: page.height - y - steps.height - 12
        radius: Theme.radius
        color: Theme.card
        border.color: Theme.border

        // CARLA 카메라 영상이 들어오면 실제 렌더링 화면, 없거나 DEMO 주행이면 그래픽 도로 화면
        // (가상 DEMO 주행만 그래픽 화면, DEMO · CARLA / LIVE 는 CARLA 실제 영상. R 기어면 후방 영상이 올 때만)
        readonly property bool useCam: !!(hmi.camera && hmi.camera.fresh)
                                       && !(page.s.demo_drive && !page.s.demo_carla)
                                       && (page.s.gear !== "R" || hmi.camera.view === "rear")
        CameraView {
            visible: parent.useCam
            anchors.fill: parent
            anchors.margins: 6
            cam: hmi.camera
            level: page.s.risk_level
            distance: page.s.distance_m
            rearDistance: page.s.rear_distance_m
            braking: (page.s.brake || 0) > 3
        }
        DriveVis3D {
            visible: !parent.useCam
            anchors.fill: parent
            anchors.margins: 6
            camView: "driver"
            reverse: page.s.gear === "R"              // 후진 기어 → 후방 카메라
            rearDistance: page.s.rear_distance_m
            world: page.world
            msaa: SceneEnvironment.Medium
        }
        Rectangle { anchors.fill: parent; color: "transparent"; radius: Theme.radius; border.width: 8; border.color: Theme.card }
        Rectangle { anchors.fill: parent; color: "transparent"; radius: Theme.radius; border.color: Theme.border }
    }

    Row {
        id: steps
        anchors.bottom: parent.bottom
        width: page.width
        height: 64
        spacing: 12
        Repeater {
            model: [["L0", "정상"], ["L1", "주의"], ["L2", "경고"], ["L3", "위험"]]
            Rectangle {
                readonly property bool on: page.s.risk_level === index
                width: (steps.width - 36) / 4
                height: steps.height
                radius: 14
                color: on ? Theme.levelColor(index) : Theme.card
                border.color: on ? "transparent" : Theme.border
                Text {
                    anchors.centerIn: parent
                    text: modelData[0] + "  " + modelData[1]
                    font.pixelSize: 24
                    font.weight: parent.on ? Font.Black : Font.Normal
                    color: parent.on ? "white" : Theme.faint
                }
            }
        }
    }
}
