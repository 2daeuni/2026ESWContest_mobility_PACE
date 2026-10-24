import QtQuick
import QtQuick.Window
import QtQuick3D
import "."

Window {
    id: win
    width: startWidth
    height: startHeight
    visible: true
    visibility: startWindowed ? Window.Windowed : Window.FullScreen
    title: "PACE HMI"
    color: Theme.bg

    // 디자인 기준 해상도. 실제 화면 크기에 맞춰 비율 유지 스케일.
    readonly property real designW: 1600
    readonly property real designH: 900
    readonly property real uiScale: Math.min(width / designW, height / designH)

    property int tab: startTab   // 0 = PACE, 1 = 진단, 2 = 기록
    property bool driverView: startDriverView   // true = 운전자 화면 (단순), false = 상세 화면
    readonly property var s: hmi.state

    Component.onCompleted: Theme.dark = startDark
    onSChanged: {
        if (typeof s.night_mode === "boolean")
            Theme.dark = s.night_mode
    }

    // 화면 갱신 속도(FPS) 측정 → 진단 탭
    property int frames: 0
    onFrameSwapped: frames++
    Timer {
        interval: 1000; running: true; repeat: true
        onTriggered: { hmi.reportFps(win.frames); win.frames = 0 }
    }

    // 3D 주행 장면 (왼쪽 클러스터 / PACE 장면 카드 / 운전자 화면이 같이 사용)
    World3D {
        id: world3d
        carlaMap: hmi.carlaMap
        sc: win.s.scene || ({})
        speed: win.s.speed_kmh || 0
        level: win.s.risk_level || 0
        braking: (win.s.brake || 0) > 3
        distance: win.s.risk_source === "Safety ECU" ? win.s.distance_m
                  : (Theme.has(win.s.distance_m) ? win.s.distance_m : (win.s.scene ? win.s.scene.lead_gap : null))
    }
    FrameAnimation {
        running: (win.s.speed_kmh || 0) > 0.3
        onTriggered: world3d.odo = (world3d.odo + (win.s.speed_kmh || 0) / 3.6 * Math.min(frameTime, 0.1)) % 900
    }

    Item {
        id: canvas
        width: win.designW
        height: win.designH
        anchors.centerIn: parent
        scale: win.uiScale

        // ================= 왼쪽 클러스터 =================
        LeftCluster {
            world: world3d
            x: 14; y: 10
            width: 520; height: 776
            s: win.s
            mode: hmi.mode
        }

        // ================= 오른쪽 =================
        Rectangle {
            id: right
            x: 544; y: 10
            width: win.designW - 544 - 14
            height: 776
            radius: 28
            color: Theme.panel
            border.color: Theme.border

            // ---- 헤더 ----
            Item {
                id: header
                x: 0; y: 0
                width: parent.width; height: 62

                Rectangle {   // 홈
                    x: 18; y: 12; width: 40; height: 40; radius: 10
                    color: "transparent"
                    Shape_Home { anchors.centerIn: parent }
                    MouseArea { anchors.fill: parent; onClicked: win.tab = 0 }
                }

                Row {
                    x: 70; y: 0
                    height: parent.height
                    Repeater {
                        model: ["PACE", "Diagnostics", "기록"]
                        Item {
                            width: 124; height: parent.height
                            Text {
                                anchors.centerIn: parent
                                text: modelData
                                font.pixelSize: 19
                                font.weight: win.tab === index ? Font.Bold : Font.Normal
                                color: win.tab === index ? Theme.text : Theme.sub
                            }
                            Rectangle {
                                visible: win.tab === index
                                anchors.bottom: parent.bottom
                                anchors.horizontalCenter: parent.horizontalCenter
                                width: 96; height: 3; radius: 2
                                color: Theme.blue
                            }
                            MouseArea { anchors.fill: parent; onClicked: win.tab = index }
                        }
                    }
                }

                Row {
                    anchors.right: parent.right
                    anchors.rightMargin: 20
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 14
                    Chip {
                        anchors.verticalCenter: parent.verticalCenter
                        text: win.driverView ? "운전자 화면" : "상세 화면"
                        selected: win.driverView
                        onClicked: { win.driverView = !win.driverView; win.tab = 0 }
                    }
                    Chip {
                        anchors.verticalCenter: parent.verticalCenter
                        text: typeof win.s.night_mode === "boolean" ? (win.s.night_mode ? "야간" : "주간") : "주야간 --"
                        selected: win.s.night_mode === true
                    }
                    Chip {
                        anchors.verticalCenter: parent.verticalCenter
                        text: win.s.sound_muted ? "소리 OFF" : "소리 ON"
                        selected: !win.s.sound_muted
                        onClicked: hmi.toggleMute()
                    }
                    Repeater {
                        model: [["CAN", hmi.links.can ? hmi.links.can.ok && win.s.link_can : false],
                                ["SAFETY", win.s.link_safety_udp]]
                        Row {
                            spacing: 6
                            Rectangle { width: 9; height: 9; radius: 5; anchors.verticalCenter: parent.verticalCenter; color: modelData[1] ? Theme.green : Theme.red }
                            Text { text: modelData[0]; font.pixelSize: 13; color: Theme.sub }
                        }
                    }
                    Text {
                        id: clock
                        font.pixelSize: 20
                        font.weight: Font.DemiBold
                        color: Theme.text
                        text: Qt.formatTime(new Date(), "hh:mm:ss")
                        Timer {
                            interval: 500; running: true; repeat: true
                            onTriggered: clock.text = Qt.formatTime(new Date(), "hh:mm:ss")
                        }
                    }
                }

                Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 1; color: Theme.border }
            }

            // ---- 본문 ----
            PacePage {
                world: world3d
                visible: win.tab === 0 && !win.driverView
                x: 12; y: header.height + 10
                width: parent.width - 24
                height: parent.height - y - 12
                s: win.s
                events: hmi.events
                onOpenDiagnostics: win.tab = 1
            }

            DriverPage {
                world: world3d
                visible: win.tab === 0 && win.driverView
                x: 12; y: header.height + 10
                width: parent.width - 24
                height: parent.height - y - 12
                s: win.s
            }

            RecordsPage {
                visible: win.tab === 2
                x: 12; y: header.height + 10
                width: parent.width - 24
                height: parent.height - y - 12
                trials: hmi.trials
                summary: hmi.trialSummary
                measuring: win.s.measuring === true
                muted: win.s.sound_muted === true
            }

            TrialToast {
                id: toast
                z: 40
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 24
            }
            Connections {
                target: hmi
                function onTrialFinished(t) { toast.show(t) }
            }

            DiagnosticsPage {
                visible: win.tab === 1
                x: 12; y: header.height + 10
                width: parent.width - 24
                height: parent.height - y - 12
                s: win.s
                diag: hmi.diag
                events: hmi.events
                links: hmi.links
                perf: hmi.perf
            }
        }

        // ================= 하단 도크 =================
        Dock {
            x: 14; y: 796
            width: win.designW - 28; height: 94
            s: win.s
            tab: win.tab
            onTabRequested: (index) => win.tab = index
        }

        // 화면 테두리 깜빡임 (주변시야 경고) - 아무것도 가리지 않음
        Rectangle {
            anchors.fill: parent
            color: "transparent"
            border.color: Theme.red
            border.width: 0
            visible: win.s.brake_now === true || win.s.risk_level === 3
            SequentialAnimation on border.width {
                running: win.s.brake_now === true || win.s.risk_level === 3
                loops: Animation.Infinite
                NumberAnimation { to: 10; duration: 260 }
                NumberAnimation { to: 2; duration: 260 }
            }
            z: 50
        }

        WarningBanner {
            width: win.designW * 0.86
            height: win.designH * 0.20
            anchors.centerIn: parent
            banner: win.s.banner
            prominent: true
            visible: win.s.banner
                     && (win.s.banner.severity === "caution"
                         || win.s.banner.severity === "warning"
                         || win.s.banner.severity === "danger")
                     && win.s.brake_now !== true
                     && win.s.demo_collided !== true
            z: 55
        }

        BrakeNowOverlay {
            anchors.fill: parent           // 화면 전체 전환
            s: win.s
            active: win.s.brake_now === true
            z: 60
        }

        CollisionOverlay {
            z: 70
            anchors.fill: parent
            active: win.s.demo_collided === true
            carla: win.s.demo_carla === true
            withWhat: win.s.banner && win.s.banner.subtitle ? win.s.banner.subtitle.replace("DEMO · CARLA — Collision ", "") : ""
        }
    }

    // 키보드: Esc 종료, F11 전체화면, Tab 탭 전환
    //        DEMO 주행: ↑/W 가속, ↓/S/Space 브레이크, R 리셋, F2 DEMO 켜기/끄기
    Item {
        id: keys
        focus: true
        function pedalKey(key) {
            if (key === Qt.Key_Up || key === Qt.Key_W) return "accel"
            if (key === Qt.Key_Down || key === Qt.Key_S || key === Qt.Key_Space) return "brake"
            return ""
        }
        Keys.onEscapePressed: Qt.quit()
        Keys.onPressed: (event) => {
            var p = pedalKey(event.key)
            if (p !== "") {
                if (!event.isAutoRepeat) hmi.setKey(p, true)
                event.accepted = true
            } else if (event.key === Qt.Key_F11) {
                win.visibility = win.visibility === Window.FullScreen ? Window.Windowed : Window.FullScreen
            } else if (event.key === Qt.Key_Tab) {
                win.tab = (win.tab + 1) % 3
            } else if (event.key === Qt.Key_R) {
                hmi.demoReset()
            } else if (event.key === Qt.Key_F2) {
                hmi.toggleDemo()
            } else if (event.key === Qt.Key_M) {
                hmi.toggleMute()
            } else if (event.key === Qt.Key_V) {
                win.driverView = !win.driverView
            }
        }
        Keys.onReleased: (event) => {
            var p = pedalKey(event.key)
            if (p !== "" && !event.isAutoRepeat) {
                hmi.setKey(p, false)
                event.accepted = true
            }
        }
    }
    // 마우스 클릭 후에도 키 입력 유지
    onActiveFocusItemChanged: if (!activeFocusItem) keys.forceActiveFocus()
}
