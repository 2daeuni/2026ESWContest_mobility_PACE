import QtQuick
import QtQuick3D
import "."

Item {
    id: page
    property var s: ({})
    property var world: null
    property var events: []
    signal openDiagnostics()

    readonly property real gap: 10
    readonly property real col3: (width - 2 * gap) / 3

    // =========================================================
    // Row 1 : 경고 배너 + 도로 장면 | PACE STATUS
    // =========================================================
    Rectangle {
        id: sceneCard
        x: 0; y: 0
        width: 540; height: 300
        radius: Theme.radius
        color: Theme.card
        border.color: Theme.border

        // CARLA 카메라 영상이 들어오면 실제 렌더링 화면, 없거나 DEMO 주행이면 그래픽 도로 화면
        readonly property bool useCam: !!(hmi.camera && hmi.camera.fresh) && !page.s.demo_drive
        CameraView {
            visible: parent.useCam
            anchors.fill: parent
            anchors.margins: 6
            cam: hmi.camera
            level: page.s.risk_level
            distance: page.s.distance_m
            braking: (page.s.brake || 0) > 3
        }
        DriveVis3D {
            visible: !parent.useCam
            anchors.fill: parent
            anchors.margins: 6
            camView: "driver"
            labelTop: 84
            world: page.world
            msaa: SceneEnvironment.Medium
        }
        // 둥근 모서리 마스크 (OpacityMask 없이 가볍게)
        Rectangle {
            anchors.fill: parent
            color: "transparent"
            radius: Theme.radius
            border.width: 8
            border.color: Theme.card
        }
        Rectangle {
            anchors.fill: parent
            color: "transparent"
            radius: Theme.radius
            border.color: Theme.border
        }

        WarningBanner {
            x: 12; y: 12
            width: parent.width - 24
            height: 74
            banner: page.s.banner
            ttc: page.s.ttc_s
        }
    }

    StatusCard {
        x: sceneCard.width + page.gap; y: 0
        width: page.width - x; height: 300
        s: page.s
    }

    // =========================================================
    // Row 2 : PEDAL | MOTOR COMMAND | HAPTIC
    // =========================================================
    Card {
        id: pedalCard
        x: 0; y: 310
        width: page.col3; height: 172
        title: "PEDAL"
        subtitle: "(CAN 0x100)"
        Column {
            anchors.fill: parent
            BarRow {
                height: 29
                label: "Accelerator"
                ratio: (page.s.accelerator || 0) / 100
                valueText: Theme.fmt(page.s.accelerator)
                unit: "%"
                barColor: page.s.brake_now ? Theme.red : Theme.green
            }
            BarRow {
                height: 29
                label: "Rate"
                ratio: Math.abs(page.s.accelerator_rate || 0) / 300
                valueText: Theme.fmt(page.s.accelerator_rate)
                unit: "%/s"
                barColor: Theme.green
            }
            BarRow {
                height: 29
                label: "Brake"
                ratio: (page.s.brake || 0) / 100
                valueText: Theme.fmt(page.s.brake)
                unit: "%"
                barColor: Theme.blue
            }
            BarRow {
                height: 29
                label: "Rate"
                ratio: Math.abs(page.s.brake_rate || 0) / 300
                valueText: Theme.fmt(page.s.brake_rate)
                unit: "%/s"
                barColor: Theme.blue
            }
        }
    }

    Card {
        id: motorCard
        x: page.col3 + page.gap; y: 310
        width: page.col3; height: 172
        title: "MOTOR"
        subtitle: "(명령 0x120 · 상태 0x210)"
        Column {
            anchors.fill: parent
            BarRow {
                height: 29
                label: "명령 제한"
                ratio: (Theme.has(page.s.motor_limit) ? page.s.motor_limit : 0) / 100
                valueText: Theme.fmt(page.s.motor_limit)
                unit: "%"
                barColor: page.s.motor_limited ? Theme.blue : Theme.green
            }
            BarRow {
                height: 29
                label: "적용 제한"
                ratio: (Theme.has(page.s.motor_applied_limit) ? page.s.motor_applied_limit : 0) / 100
                valueText: Theme.fmt(page.s.motor_applied_limit)
                unit: "%"
                barColor: Theme.blue
            }
            BarRow {
                height: 29
                label: "실제 PWM"
                ratio: (Theme.has(page.s.motor_pwm) ? page.s.motor_pwm : 0) / 100
                valueText: Theme.fmt(page.s.motor_pwm)
                unit: "%"
                barColor: Theme.green
            }
            Rectangle {
                width: parent.width; height: 30
                radius: 8
                color: Theme.cardSoft
                // 상태 우선순위: 0x210 고장/E-STOP > 0x120 명령
                readonly property var faults: page.s.motor_faults || []
                readonly property string st: page.s.motor_state || ""
                readonly property string m: page.s.motor_mode || "-"
                readonly property bool bad: faults.length > 0 || st === "FAULT" || m === "STOP"
                readonly property string label: faults.indexOf("ESTOP_ACTIVE") >= 0 ? "E-STOP 중 · 페달 떼면 복구"
                    : faults.length > 0 ? "고장: " + faults.join(", ")
                    : st === "FAULT" ? "모터 고장"
                    : m === "STOP" ? (st === "" ? "정지 명령 · 모터 ECU 응답 없음" : "정지 명령")
                    : m === "LIMIT" ? "가속 제한 중 (" + st + ")"
                    : m === "NORMAL" ? "정상 출력" + (st !== "" ? " (" + st + ")" : "")
                    : (st !== "" ? st : "명령 없음")
                Glyph {
                    id: mg
                    x: 10
                    anchors.verticalCenter: parent.verticalCenter
                    size: 18
                    kind: parent.bad ? "fault" : (parent.m === "LIMIT" ? "limit" : (parent.m === "NORMAL" ? "ok" : "info"))
                    color: parent.bad ? Theme.red : (parent.m === "LIMIT" ? Theme.blue : (parent.m === "NORMAL" ? Theme.green : Theme.gray))
                }
                Text {
                    anchors.left: mg.right
                    anchors.leftMargin: 8
                    anchors.right: parent.right
                    anchors.rightMargin: 8
                    anchors.verticalCenter: parent.verticalCenter
                    text: parent.label
                    elide: Text.ElideRight
                    font.pixelSize: 14
                    color: parent.bad ? Theme.red : Theme.text
                }
            }
        }
    }

    Card {
        id: hapticCard
        x: 2 * (page.col3 + page.gap); y: 310
        width: page.width - x; height: 172
        title: "HAPTIC"
        subtitle: "(CAN 0x110)"
        Column {
            anchors.fill: parent
            Item {
                width: parent.width; height: 29
                Text { width: 110; text: "Status"; font.pixelSize: 16; color: Theme.text; anchors.verticalCenter: parent.verticalCenter }
                Row {
                    x: 110
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 8
                    readonly property bool on: page.s.haptic_cmd_on === true
                    readonly property bool fault: Theme.has(page.s.haptic_fault) && page.s.haptic_fault !== "NO_FAULT"
                    Rectangle {
                        width: 12; height: 12; radius: 6
                        anchors.verticalCenter: parent.verticalCenter
                        color: parent.fault ? Theme.red : (parent.on ? Theme.blue : Theme.gray)
                    }
                    Text {
                        text: parent.fault ? "Fault: " + page.s.haptic_fault
                              : (parent.on ? "Active" : (page.s.haptic_cmd_on === false ? "Off" : "No Data"))
                        font.pixelSize: 16
                        color: parent.fault ? Theme.red : (parent.on ? Theme.blue : Theme.sub)
                    }
                }
            }
            BarRow {
                height: 29
                label: "Intensity"
                ratio: (page.s.haptic_cmd_intensity || 0) / 100
                valueText: Theme.fmt(page.s.haptic_cmd_intensity)
                unit: "%"
                gradient: true
            }
            BarRow {
                height: 29
                label: "Frequency"
                ratio: (page.s.haptic_cmd_frequency || 0) / 50
                valueText: Theme.fmt(page.s.haptic_cmd_frequency)
                unit: "Hz"
                gradient: true
            }
            Row {
                height: 29
                spacing: 8
                visible: page.s.haptic_cmd_on === true || page.s.haptic_ready === false
                Glyph {
                    anchors.verticalCenter: parent.verticalCenter
                    kind: "vib"; size: 22
                    color: page.s.haptic_ready === false ? Theme.red : Theme.orange
                }
                Text {
                    anchors.verticalCenter: parent.verticalCenter
                    text: page.s.haptic_ready === false ? "햅틱 ECU 준비 안 됨"
                          : (page.s.haptic_vibrating === true ? "페달 진동 경고 중" : "진동 명령 송신 중")
                    font.pixelSize: 15
                    color: page.s.haptic_ready === false ? Theme.red : Theme.orange
                }
            }
        }
    }

    // =========================================================
    // Row 3 : SAFETY | SYSTEM STATUS | EVENT LOG
    // =========================================================
    Card {
        x: 0; y: 492
        width: page.col3; height: page.height - 492
        id: safetyCard
        title: "SAFETY"
        subtitle: "판정 근거 · 3개 중 2개"
        readonly property color lc: Theme.levelColor(page.s.risk_level)
        Column {
            anchors.fill: parent
            anchors.topMargin: -8
            spacing: 4
            ConditionMatrix {
                width: parent.width
                conditions: page.s.risk_conditions
                thresholds: page.s.risk_thresholds
                level: page.s.risk_level
            }
            Item {
                width: parent.width; height: 30
                Text { width: 84; text: "Fail-Safe"; font.pixelSize: 15; color: Theme.text; anchors.verticalCenter: parent.verticalCenter }
                Rectangle {
                    x: 84
                    anchors.verticalCenter: parent.verticalCenter
                    width: parent.width - 84; height: 28; radius: 8
                    color: page.s.failsafe_active ? Theme.redSoft : Theme.greenSoft
                    Text {
                        anchors.left: parent.left
                        anchors.leftMargin: 12
                        anchors.right: parent.right
                        anchors.rightMargin: 8
                        anchors.verticalCenter: parent.verticalCenter
                        text: page.s.failsafe_active ? (page.s.failsafe_reason_ko || "Active") : "Normal"
                        elide: Text.ElideRight
                        font.pixelSize: 15
                        color: page.s.failsafe_active ? Theme.red : Theme.green
                    }
                }
            }
        }
    }

    Card {
        x: page.col3 + page.gap; y: 492
        width: page.col3; height: page.height - 492
        title: "SYSTEM STATUS"
        Column {
            anchors.fill: parent
            anchors.topMargin: -4
            spacing: 1
            Repeater {
                model: [
                    ["CAN (PEDAL)", page.s.link_pedal],
                    ["Safety ECU", page.s.link_safety],
                    ["HAPTIC (ECU)", page.s.link_haptic],
                    ["MOTOR (ECU)", page.s.link_motor],
                    ["CARLA (Scene)", page.s.link_carla]
                ]
                Item {
                    width: parent.width; height: 20
                    Text { text: modelData[0]; font.pixelSize: 15; color: Theme.sub; anchors.verticalCenter: parent.verticalCenter }
                    Row {
                        x: parent.width * 0.52
                        anchors.verticalCenter: parent.verticalCenter
                        spacing: 9
                        Rectangle {
                            width: 11; height: 11; radius: 6
                            anchors.verticalCenter: parent.verticalCenter
                            color: Theme.linkColor(modelData[1])
                        }
                        Text {
                            text: Theme.linkText(modelData[1])
                            font.pixelSize: 15
                            color: modelData[1] === false ? Theme.red : Theme.sub
                        }
                    }
                }
            }
        }
    }

    Card {
        x: 2 * (page.col3 + page.gap); y: 492
        width: page.width - x; height: page.height - 492
        title: "EVENT LOG"
        headerRight: Text {
            text: "더보기 ›"
            font.pixelSize: 14
            color: Theme.sub
            y: 4
            MouseArea { anchors.fill: parent; anchors.margins: -10; onClicked: page.openDiagnostics() }
        }
        Column {
            anchors.fill: parent
            anchors.topMargin: -4
            spacing: 2
            Repeater {
                model: page.events.slice(0, 5)
                Item {
                    width: parent.width; height: 22
                    Text {
                        id: t
                        text: modelData.time
                        font.pixelSize: 14
                        color: Theme.sub
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Glyph {
                        id: ic
                        x: t.width + 14
                        anchors.verticalCenter: parent.verticalCenter
                        size: 17
                        kind: modelData.severity === "danger" || modelData.severity === "fault" ? "warn"
                              : (modelData.severity === "warning" ? "vib"
                              : (modelData.severity === "ok" ? "ok" : "info"))
                        color: Theme.severityColor(modelData.severity)
                    }
                    Text {
                        anchors.left: ic.right
                        anchors.leftMargin: 10
                        anchors.right: parent.right
                        anchors.verticalCenter: parent.verticalCenter
                        text: modelData.text
                        font.pixelSize: 14
                        color: Theme.text
                        elide: Text.ElideRight
                    }
                }
            }
        }
    }
}
