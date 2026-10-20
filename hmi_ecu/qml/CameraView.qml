import QtQuick
import "."

// CARLA 가 실제로 렌더링한 추격 카메라 영상 + AR 오버레이
//   - 영상: hmi.camera (TCP 5602, carla_tools/camera_stream.py)
//   - 자차 주행 경로(차선): 위험 단계 색으로 바닥에 깔림, 앞차 뒤에서 끊김
//   - 앞차: 모서리 브래킷 + 거리 표시 (L2 이상이면 강조)
// 오버레이 좌표는 브리지가 같은 카메라 행렬로 투영한 이미지 픽셀 좌표 → 화면 크기에 맞춰 변환만 함
Item {
    id: view
    property var cam: ({})
    property var level: 0
    property var distance: null        // 화면에 표시할 앞차 거리 (Safety ECU 값 우선)
    property bool braking: false
    property var rearDistance: null    // R 기어: 후방 장애물 거리
    readonly property bool rear: cam.view === "rear"
    clip: true

    readonly property real iw: cam.w || 16
    readonly property real ih: cam.h || 9
    readonly property real sc: Math.max(width / iw, height / ih)       // PreserveAspectCrop 과 동일
    readonly property real ox: (width - iw * sc) / 2
    readonly property real oy: (height - ih * sc) / 2
    readonly property var ov: cam.overlay || ({})
    readonly property color pathColor: (level === null || level === undefined || level === 0)
                                       ? Theme.blue : Theme.levelColor(level)

    Rectangle { anchors.fill: parent; color: "#0E1116" }

    Image {
        id: frame
        anchors.fill: parent
        fillMode: Image.PreserveAspectCrop
        cache: false
        smooth: true
        asynchronous: false
        source: view.cam.frameId ? "image://carla/" + view.cam.frameId : ""
    }

    // ---------- AR 오버레이 ----------
    Canvas {
        id: overlay
        anchors.fill: parent
        renderStrategy: Canvas.Cooperative

        function mx(p) { return view.ox + p[0] * view.sc }
        function my(p) { return view.oy + p[1] * view.sc }

        onPaint: {
            var ctx = getContext("2d")
            ctx.reset()
            var o = view.ov
            if (view.rear) {
                // 후진 가이드선 (빨강 0.5 m · 노랑 1 m · 초록 3 m)
                var G = o.guides || []
                ctx.lineCap = "round"
                for (var gi = 0; gi < G.length; gi++) {
                    var gp = G[gi].pts
                    if (!gp || gp.length < 2) continue
                    ctx.lineWidth = Math.max(2.5, 4 * view.sc)
                    ctx.strokeStyle = G[gi].c === "red" ? "#FF3B30" : (G[gi].c === "yellow" ? "#FFCC00" : "#34C759")
                    ctx.beginPath()
                    ctx.moveTo(mx(gp[0]), my(gp[0]))
                    for (var gj = 1; gj < gp.length; gj++) ctx.lineTo(mx(gp[gj]), my(gp[gj]))
                    ctx.stroke()
                }
                return
            }
            var L = o.lane_left || [], R = o.lane_right || []
            var lead = o.lead_box

            // 1) 자차 경로: 앞차 뒷부분에서 끊기게
            if (L.length > 1 && R.length > 1) {
                ctx.save()
                if (lead) {
                    ctx.beginPath()
                    ctx.rect(0, view.oy + lead[3] * view.sc - 2, width, height)
                    ctx.clip()
                }
                var top = height
                for (var i = 0; i < L.length; i++) top = Math.min(top, my(L[i]))
                var g = ctx.createLinearGradient(0, height, 0, top)
                var c = view.pathColor
                g.addColorStop(0.0, Qt.rgba(c.r, c.g, c.b, 0.55))
                g.addColorStop(0.65, Qt.rgba(c.r, c.g, c.b, 0.30))
                g.addColorStop(1.0, Qt.rgba(c.r, c.g, c.b, 0.0))

                ctx.beginPath()
                ctx.moveTo(mx(L[0]), my(L[0]))
                for (i = 1; i < L.length; i++) ctx.lineTo(mx(L[i]), my(L[i]))
                for (i = R.length - 1; i >= 0; i--) ctx.lineTo(mx(R[i]), my(R[i]))
                ctx.closePath()
                ctx.fillStyle = g
                ctx.fill()

                // 경계선 (가늘게)
                ctx.lineWidth = Math.max(1.5, 2.2 * view.sc)
                ctx.strokeStyle = Qt.rgba(c.r, c.g, c.b, 0.85)
                var edges = [L, R]
                for (var e = 0; e < 2; e++) {
                    var P = edges[e]
                    ctx.beginPath()
                    ctx.moveTo(mx(P[0]), my(P[0]))
                    for (i = 1; i < P.length; i++) ctx.lineTo(mx(P[i]), my(P[i]))
                    ctx.stroke()
                }
                ctx.restore()
            }

            // 2) 앞차 강조
            if (lead) {
                var x0 = view.ox + lead[0] * view.sc, y0 = view.oy + lead[1] * view.sc
                var x1 = view.ox + lead[2] * view.sc, y1 = view.oy + lead[3] * view.sc
                var bw = x1 - x0, bh = y1 - y0
                var pad = Math.max(3, bw * 0.06)
                x0 -= pad; y0 -= pad; x1 += pad; y1 += pad; bw = x1 - x0; bh = y1 - y0
                var lc = (view.level >= 1) ? Theme.levelColor(view.level) : Qt.rgba(1, 1, 1, 1)
                if (view.level >= 2) {
                    ctx.fillStyle = Qt.rgba(lc.r, lc.g, lc.b, view.level >= 3 ? 0.28 : 0.16)
                    ctx.fillRect(x0, y0, bw, bh)
                }
                var arm = Math.max(8, Math.min(bw, bh) * 0.28)
                ctx.lineWidth = view.level >= 2 ? 3.5 : 2.5
                ctx.strokeStyle = lc
                ctx.lineCap = "round"
                ctx.beginPath()
                ctx.moveTo(x0, y0 + arm); ctx.lineTo(x0, y0); ctx.lineTo(x0 + arm, y0)
                ctx.moveTo(x1 - arm, y0); ctx.lineTo(x1, y0); ctx.lineTo(x1, y0 + arm)
                ctx.moveTo(x1, y1 - arm); ctx.lineTo(x1, y1); ctx.lineTo(x1 - arm, y1)
                ctx.moveTo(x0 + arm, y1); ctx.lineTo(x0, y1); ctx.lineTo(x0, y1 - arm)
                ctx.stroke()
            }
        }
    }

    // 앞차 거리 라벨 (Text 로 그려야 한글/폰트가 깔끔)
    Rectangle {
        id: gapPill
        readonly property var lead: view.ov.lead_box
        readonly property var gapM: Theme.has(view.distance) ? view.distance : view.ov.lead_gap
        visible: !view.rear && !!lead && Theme.has(gapM)
        x: lead ? Math.max(6, Math.min(view.width - width - 6, view.ox + (lead[0] + lead[2]) / 2 * view.sc - width / 2)) : 0
        y: lead ? Math.max(6, view.oy + lead[1] * view.sc - height - 10) : 0
        width: gapText.width + 18; height: 26; radius: 13
        color: view.level >= 1 ? Theme.levelColor(view.level) : Qt.rgba(0.07, 0.08, 0.1, 0.72)
        Text {
            id: gapText
            anchors.centerIn: parent
            text: parent.gapM === undefined || parent.gapM === null ? "" : parent.gapM.toFixed(parent.gapM < 10 ? 1 : 0) + " m"
            font.pixelSize: 14
            font.weight: Font.Bold
            color: "white"
        }
    }

    // ---------- 후진: R 태그 + 후방 거리 ----------
    Row {
        visible: view.rear
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 12
        spacing: 8
        Rectangle {
            width: rTag.width + 16; height: 26; radius: 13
            color: Qt.rgba(0.07, 0.08, 0.1, 0.72)
            Text { id: rTag; anchors.centerIn: parent; text: "R  후방"; font.pixelSize: 13; font.weight: Font.Bold; color: "white" }
        }
        Rectangle {
            id: rearPill
            readonly property var gapM: Theme.has(view.rearDistance) ? view.rearDistance : view.ov.rear_gap
            visible: Theme.has(gapM) && gapM < 20
            width: rearText.width + 18; height: 26; radius: 13
            color: !visible ? "transparent" : (gapM < 0.6 ? "#FF3B30" : (gapM < 1.2 ? "#FF9500" : (gapM < 2.0 ? "#E0B000" : Qt.rgba(0.2, 0.68, 0.35, 0.92))))
            SequentialAnimation on opacity {
                running: rearPill.visible && rearPill.gapM < 0.6
                loops: Animation.Infinite
                NumberAnimation { to: 0.35; duration: 250 }
                NumberAnimation { to: 1.0; duration: 250 }
                onRunningChanged: if (!running) rearPill.opacity = 1
            }
            Text {
                id: rearText
                anchors.centerIn: parent
                text: Theme.has(parent.gapM) ? "후방 " + parent.gapM.toFixed(parent.gapM < 10 ? 1 : 0) + " m" : ""
                font.pixelSize: 14; font.weight: Font.Bold; color: "white"
            }
        }
    }

    // 위험 단계 L3: 화면 가장자리 붉은 비네트
    Rectangle {
        anchors.fill: parent
        visible: view.level >= 3
        color: "transparent"
        border.width: 6
        border.color: Qt.rgba(Theme.red.r, Theme.red.g, Theme.red.b, 0.75)
    }

    // 위쪽 경고 배너가 영상 위에서도 잘 읽히게 살짝 어둡게
    Rectangle {
        anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
        height: parent.height * 0.32
        gradient: Gradient {
            GradientStop { position: 0.0; color: Qt.rgba(0, 0, 0, 0.35) }
            GradientStop { position: 1.0; color: Qt.rgba(0, 0, 0, 0.0) }
        }
    }

    // ---------- 출처 태그 ----------
    Rectangle {
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 10
        width: camTag.width + 16; height: 22; radius: 11
        color: Qt.rgba(0.07, 0.08, 0.1, 0.7)
        Row {
            id: camTag
            anchors.centerIn: parent
            spacing: 6
            Rectangle { width: 7; height: 7; radius: 3.5; color: "#FF4D4D"; anchors.verticalCenter: parent.verticalCenter }
            Text {
                text: (view.rear ? "CARLA 후방 CAM" : "CARLA CAM") + (view.cam.fps ? "  " + view.cam.fps.toFixed(0) + " fps" : "")
                font.pixelSize: 12
                font.weight: Font.Bold
                color: "white"
            }
        }
    }

    Connections {
        target: view
        function onCamChanged() { overlay.requestPaint() }
        function onLevelChanged() { overlay.requestPaint() }
        function onWidthChanged() { overlay.requestPaint() }
        function onHeightChanged() { overlay.requestPaint() }
    }
}
