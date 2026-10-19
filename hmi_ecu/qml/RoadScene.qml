import QtQuick
import QtQuick.Shapes
import "."

// 원근 도로 장면 (Tesla 주행 화면 방식)
// hmi.state.scene 의 자차 기준 좌표(m: x 앞, y 오른쪽)를 추격 카메라 시점으로 투영해서 그림
//   - CARLA : 실제 주변 차량 / 보행자 / 곡선 차선
//   - DEMO  : 가상 전방 차량
//   - SENSOR: 거리값만 있을 때 정면 앞차
Item {
    id: scene
    property var sc: ({})
    property var speed: null           // km/h (차선 흐름 애니메이션)
    property var level: null
    property bool braking: false

    readonly property color tint: Theme.levelColor(level)
    readonly property color egoLaneColor: Theme.has(level) && level > 0 ? tint : Theme.blue

    // ---------- 카메라 ----------
    // Tesla 화면처럼: 자차 바로 뒤·높은 곳에서 내려다보고, 앞 70 m 정도만 그림
    property real camBack: 18.0         // 자차 중심에서 뒤로 (m). 클수록 원근이 완만 (먼 차도 크게 보임)
    property real camH: 7.5             // 높이 (m)  (높을수록 위에서 내려다봄)
    property real farLimit: 70          // 이보다 먼 것은 그리지 않음 (m)
    property real horizonRatio: 0.12
    readonly property real horizon: height * horizonRatio
    readonly property real cx: width / 2
    readonly property real egoLen: sc && sc.ego ? sc.ego.length : 4.7
    readonly property real egoWid: sc && sc.ego ? sc.ego.width : 1.85
    // 자차 뒷범퍼 바닥이 화면 아래 끝에서 6px 위에 오도록 초점거리 결정
    readonly property real f: (height - 6 - horizon) * (camBack - egoLen / 2) / camH

    function zOf(x) { return x + camBack }
    function sx(x, y) { return cx + f * y / zOf(x) }
    function sy(x) { return horizon + f * camH / zOf(x) }

    // 폴리라인(m) → 화면 점 (카메라 앞쪽만)
    function projectLine(pts) {
        var out = []
        if (!pts) return out
        for (var i = 0; i < pts.length; i++) {
            var x = pts[i][0], y = pts[i][1]
            if (zOf(x) < 1.0) continue
            if (x > farLimit || Math.abs(y) > 30) break      // 멀거나 너무 옆이면 거기서 끊음
            if (i > 0) {                                     // 교차로에서 45° 넘게 꺾이는 구간은 안 그림
                var dx = x - pts[i - 1][0], dy = y - pts[i - 1][1]
                if (dx <= 0 || Math.abs(dy) > dx) break
            }
            out.push(Qt.point(sx(x, y), sy(x)))
        }
        return out
    }

    clip: true

    // ---------- 하늘/배경 ----------
    Rectangle {
        anchors.fill: parent
        gradient: Gradient {
            GradientStop { position: 0.0; color: Theme.sky1 }
            GradientStop { position: 0.32; color: Theme.sky2 }
            GradientStop { position: 1.0; color: Theme.sky3 }
        }
    }

    // ---------- 도로면 ----------
    Shape {
        anchors.fill: parent
        preferredRendererType: Shape.CurveRenderer
        ShapePath {
            strokeColor: "transparent"
            fillGradient: LinearGradient {
                x1: 0; y1: scene.horizon; x2: 0; y2: scene.height
                GradientStop { position: 0; color: Theme.road1 }
                GradientStop { position: 1; color: Theme.road2 }
            }
            PathPolyline {
                path: {
                    var l = scene.projectLine(scene.sc ? scene.sc.road_left : null)
                    var r = scene.projectLine(scene.sc ? scene.sc.road_right : null)
                    return l.concat(r.reverse())
                }
            }
        }
    }

    // ---------- 차선 흐름 (속도감) ----------
    property real dashOffset: 0
    Timer {
        interval: 33; repeat: true
        running: Theme.has(scene.speed) && scene.speed > 0.5
        onTriggered: scene.dashOffset = (scene.dashOffset - scene.speed * 0.012) % 1000
    }

    // ---------- 위험 영역 (자차 ~ 앞차) ----------
    Shape {
        anchors.fill: parent
        visible: !!(Theme.has(scene.level) && scene.level > 0 && scene.sc && scene.sc.risk_zone && scene.sc.risk_zone.length > 2)
        opacity: scene.level === 3 ? 0.45 : (scene.level === 2 ? 0.32 : 0.2)
        preferredRendererType: Shape.CurveRenderer
        ShapePath {
            strokeColor: "transparent"
            fillColor: scene.tint
            PathPolyline { path: scene.projectLine(scene.sc ? scene.sc.risk_zone : null) }
        }
    }

    // ---------- 자차 차선 바닥 (Tesla 처럼 은은한 주행 경로) ----------
    Shape {
        anchors.fill: parent
        opacity: 0.16
        preferredRendererType: Shape.CurveRenderer
        ShapePath {
            strokeColor: "transparent"
            fillColor: scene.egoLaneColor
            PathPolyline {
                path: {
                    if (!scene.sc || !scene.sc.lines || scene.sc.lines.length < 2) return []
                    var l = scene.projectLine(scene.sc.lines[0].points)
                    var r = scene.projectLine(scene.sc.lines[1].points)
                    var n = Math.min(l.length, r.length)
                    return l.slice(0, n).concat(r.slice(0, n).reverse())
                }
            }
        }
    }

    // ---------- 차선 (고정 개수 delegate → 매 프레임 재생성 없음) ----------
    Repeater {
        model: 8
        Shape {
            readonly property var line: (scene.sc && scene.sc.lines && index < scene.sc.lines.length) ? scene.sc.lines[index] : null
            anchors.fill: parent
            visible: line !== null
            preferredRendererType: Shape.CurveRenderer
            ShapePath {
                strokeColor: !line ? "transparent" : (line.kind === "ego" ? scene.egoLaneColor : Theme.laneLine)
                strokeWidth: !line ? 0 : (line.kind === "ego" ? 4 : (line.kind === "solid" ? 4 : 3))
                strokeStyle: line && line.kind === "dash" ? ShapePath.DashLine : ShapePath.SolidLine
                dashPattern: [5, 6]
                dashOffset: scene.dashOffset
                fillColor: "transparent"
                capStyle: ShapePath.RoundCap
                joinStyle: ShapePath.RoundJoin
                PathPolyline { path: line ? scene.projectLine(line.points) : [] }
            }
        }
    }

    // ---------- 먼 곳 안개 (지평선 근처를 부드럽게 흐림) ----------
    Rectangle {
        x: 0; width: parent.width
        y: scene.horizon - 2
        height: (scene.height - scene.horizon) * 0.32
        gradient: Gradient {
            GradientStop { position: 0.0; color: Theme.sky2 }
            GradientStop { position: 1.0; color: "transparent" }
        }
    }

    // ---------- 주변 차량 / 보행자 (먼 것부터) ----------
    Repeater {
        model: 32
        VehicleSprite {
            readonly property var veh: (scene.sc && scene.sc.vehicles && index < scene.sc.vehicles.length) ? scene.sc.vehicles[index] : null
            readonly property real rearX: veh ? veh.x - veh.length / 2 : 0
            readonly property real depth: scene.zOf(rearX)
            visible: veh !== null && depth > 2.5 && rearX < scene.farLimit
            z: 1000 - rearX   // 도로/차선 위, 가까운 차가 위에
            sprite: veh ? veh.sprite : ""
            walker: veh ? veh.kind === "walker" : false
            groundX: veh ? scene.sx(rearX, veh.y) : 0
            groundY: veh ? scene.sy(rearX) : 0
            carWidth: veh ? Math.max(6, scene.f * (walker ? 0.45 : veh.width * 1.12) / Math.max(1, depth)) : 0
            highlight: veh ? veh.lead && Theme.has(scene.level) && scene.level >= 2 : false
            highlightColor: scene.tint
            fallbackColor: veh && veh.lead && Theme.has(scene.level) && scene.level >= 2 ? "#E23B3B" : "#BFC5CD"
            brakeLights: true
        }
    }

    // ---------- 앞차 거리 라벨 ----------
    Text {
        id: gapLabel
        readonly property var leadVeh: {
            if (!scene.sc || !scene.sc.vehicles) return null
            for (var i = 0; i < scene.sc.vehicles.length; i++)
                if (scene.sc.vehicles[i].lead) return scene.sc.vehicles[i]
            return null
        }
        readonly property bool hasGap: !!(scene.sc && Theme.has(scene.sc.lead_gap))
        readonly property real rearX: leadVeh ? leadVeh.x - leadVeh.length / 2 : 0
        visible: hasGap
        z: 3000
        text: !hasGap ? "" : (scene.sc.lead_gap >= 1000 ? (scene.sc.lead_gap / 1000).toFixed(2) + " km"
                                                          : scene.sc.lead_gap.toFixed(1) + " m")
        // 앞차가 보이면 그 위, 너무 멀면 지평선 위
        x: (leadVeh && rearX < scene.farLimit ? scene.sx(rearX, leadVeh.y) : scene.cx) - width / 2
        y: leadVeh && rearX < scene.farLimit
           ? scene.sy(rearX) - scene.f * leadVeh.width * 0.95 / scene.zOf(rearX) - height - 4
           : scene.horizon - height - 2
        font.pixelSize: 17
        font.weight: Font.Bold
        color: Theme.has(scene.level) && scene.level >= 2 ? Theme.red : Theme.text
        style: Text.Outline
        styleColor: Theme.card
    }

    Text {
        visible: !scene.sc || !scene.sc.source || scene.sc.source === "NONE"
        anchors.horizontalCenter: parent.horizontalCenter
        y: scene.horizon + 16
        text: "전방 감지 데이터 없음"
        font.pixelSize: 15
        color: Theme.sub
    }

    // ---------- 자차 ----------
    VehicleSprite {
        z: 2000
        sprite: scene.sc && scene.sc.ego ? scene.sc.ego.sprite : ""
        groundX: scene.cx
        groundY: scene.sy(-scene.egoLen / 2)
        carWidth: scene.f * scene.egoWid * 1.12 / scene.zOf(-scene.egoLen / 2)
        fallbackColor: "#D5D9DF"
        brakeLights: scene.braking
    }

    // ---------- 장면 출처 ----------
    Rectangle {
        z: 3000
        visible: !!(scene.sc && scene.sc.source && scene.sc.source !== "NONE")
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 10
        width: srcText.width + 16; height: 22; radius: 11
        color: Theme.card
        opacity: 0.85
        Text {
            id: srcText
            anchors.centerIn: parent
            text: scene.sc ? scene.sc.source : ""
            font.pixelSize: 12
            font.weight: Font.Bold
            color: Theme.sub
        }
    }
}
