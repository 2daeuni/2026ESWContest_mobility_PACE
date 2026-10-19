import QtQuick
import QtQuick3D
import "."

// 테슬라식 주행 화면 (Qt Quick 3D, 라즈베리파이 5 GPU 렌더링). 장면은 World3D 를 공유
//   - 정차(주행 데이터 없음): 자차를 위에서 내려다보는 화면
//   - 주행: 자차 뒤·위 추격 시점, 주변 차량(CARLA)/전방 차량(DEMO, 센서)을 실제 위치에 3D 로 표시
Item {
    id: vis
    property var world: null              // Main.qml 의 World3D
    property bool showLabels: true
    property real labelTop: 4             // 위쪽 경고 배너에 가리지 않게 (PACE 장면 카드)
    // camView: "cluster" (왼쪽 클러스터, 테슬라처럼 뒤·위에서 보는 시점, 정차 시 위에서 내려다봄)
    //          "driver"  (운전석 눈높이 1.3 m 에서 앞을 보는 시점, 자차는 안 보임, 정차·주행 동일)
    //          "forward" / "wide" (추격 시점 변형)
    property string camView: "cluster"
    property int msaa: SceneEnvironment.High

    readonly property bool driving: world ? world.driving : false
    readonly property bool dark: Theme.dark
    readonly property var level: world ? world.level : 0
    readonly property color levelCol: Theme.levelColor(level)
    readonly property var sc: world ? world.sc : ({})

    // [카메라 높이, 카메라 z(+ 뒤 / - 앞), 바라보는 z, 세로 화각]
    readonly property var camSet: camView === "driver" ? [1.3, -2.7, -60, 42]
                                  : !driving ? [5.6, 8.6, 0.2, 26]
                                  : camView === "forward" ? [3.4, 7.5, -30, 34]
                                  : camView === "wide" ? [6.2, 9.6, -12, 42]
                                  : [5.6, 8.4, -8, 48]
    property real camH: camSet[0]
    property real camBack: camSet[1]
    property real aimZ: camSet[2]
    property real fov: camSet[3]
    Behavior on camH { NumberAnimation { duration: 900; easing.type: Easing.InOutCubic } }
    Behavior on camBack { NumberAnimation { duration: 900; easing.type: Easing.InOutCubic } }
    Behavior on aimZ { NumberAnimation { duration: 900; easing.type: Easing.InOutCubic } }
    Behavior on fov { NumberAnimation { duration: 900; easing.type: Easing.InOutCubic } }

    Rectangle { anchors.fill: parent; color: world ? world.bgColor : Theme.bg }

    // GPU(OpenGL ES 3) 가 없으면 Qt Quick 3D 가 안 그려짐 → 기존 2D 도로 화면으로 대체
    readonly property bool has3D: GraphicsInfo.api !== GraphicsInfo.Software
    Loader {
        anchors.fill: parent
        active: !vis.has3D
        sourceComponent: RoadScene {
            sc: vis.sc
            speed: vis.world ? vis.world.speed : 0
            level: vis.level
            braking: vis.world ? vis.world.braking : false
        }
    }

    View3D {
        id: v3d
        visible: vis.has3D
        anchors.fill: parent
        importScene: vis.world
        camera: cam

        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Transparent
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: vis.msaa
            lightProbe: Texture { source: vis.dark ? "../assets/env/studio_dark.hdr" : "../assets/env/studio.hdr" }
            probeExposure: 0.75
            tonemapMode: SceneEnvironment.TonemapModeLinear
            fog: Fog {
                enabled: true
                color: vis.world ? vis.world.bgColor : "white"
                depthEnabled: true
                depthNear: (vis.driving || vis.camView === "driver") ? 55 : 30
                depthFar: (vis.driving || vis.camView === "driver") ? 120 : 60
                depthCurve: 1.0
            }
        }

        PerspectiveCamera {
            id: cam
            position: Qt.vector3d(0, vis.camH, vis.camBack)
            eulerRotation.x: -Math.atan2(vis.camH - 0.4, vis.camBack - vis.aimZ) * 180 / Math.PI
            fieldOfView: vis.fov
            clipNear: 0.5
            clipFar: 400
        }
    }

    // Temporary runtime trace from the left cluster, after actual delegate bindings.
    Timer {
        interval: 500
        repeat: true
        running: vis.visible && vis.camView === "cluster" && !!vis.world && vis.sc.source === "CARLA"
        onTriggered: console.log(vis.world.debugLine()
                                 + " has3D=" + Number(vis.has3D)
                                 + " camera=" + vis.world.debugVector(cam.scenePosition)
                                 + "/" + vis.world.debugVector(cam.eulerRotation))
    }

    // 거리 라벨 (2D, 3D 위치를 화면 좌표로 변환)
    Rectangle {
        id: gapLabel
        readonly property var lead: (vis.world ? vis.world.leadAnchor.lead : null)
        readonly property var gap: vis.world ? vis.world.distance : null
        property point p: Qt.point(0, 0)
        visible: vis.showLabels && !!lead && Theme.has(gap) && vis.driving && p.y > 0 && p.y < vis.height
        x: p.x - width / 2
        y: Math.max(vis.labelTop, p.y - height - 4)
        width: gapText.width + 20; height: 26; radius: 13
        color: vis.level >= 1 ? vis.levelCol : (vis.dark ? "#2A3038" : "#FFFFFF")
        border.color: vis.level >= 1 ? "transparent" : Theme.border
        Text {
            id: gapText
            anchors.centerIn: parent
            text: !Theme.has(parent.gap) ? "" : (parent.gap >= 1000 ? (parent.gap / 1000).toFixed(2) + " km" : parent.gap.toFixed(parent.gap < 10 ? 1 : 0) + " m")
            font.pixelSize: 14
            font.weight: Font.Bold
            color: vis.level >= 1 ? "white" : Theme.text
        }
        function refresh() {
            if (!lead) return
            var q = v3d.mapFrom3DScene(vis.world.leadAnchor.scenePosition)
            p = Qt.point(q.x, q.y)
        }
    }
    FrameAnimation { running: gapLabel.lead !== null && vis.visible; onTriggered: gapLabel.refresh() }
}
