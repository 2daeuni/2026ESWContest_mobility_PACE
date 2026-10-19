import QtQuick
import QtQuick3D
import PaceHmi 1.0
import "."

// 3D 주행 장면 (한 번만 만들고 여러 화면이 importScene 으로 공유 → 형상 계산 1번, 시점만 다름)
//   좌표: 장면 dict 의 자차 기준 x 앞 / y 오른쪽 (m) → 3D X = y, Z = -x, Y 위
Node {
    id: w
    position: Qt.vector3d(0, 0, 0)
    eulerRotation: Qt.vector3d(0, 0, 0)
    property var sc: ({})
    property var speed: 0
    property var level: 0
    property bool braking: false
    property var distance: null          // 전방 거리 라벨 (Safety ECU 값 우선, 없으면 CARLA/DEMO)
    property var carlaMap: ({})          // hmi.carlaMap : CARLA 실제 도로/차선/건물 (앵커 좌표)
    readonly property bool mapOn: !!(sc && sc.has_map && carlaMap && carlaMap.ver !== undefined
                                     && carlaMap.ver !== null && carlaMap.ver === sc.map_ver)
    // Latch ROAD3D across elevated plateaus; only return at the entry height.
    property string renderMode: "MAP_MODE"
    readonly property bool slopeMode: renderMode === "ROAD3D_MODE"
    property var baseFlatZ: null
    readonly property var egoWorldZ: sc && sc.ego ? sc.ego.z : null
    function validNumber(value) {
        return typeof value === "number" && isFinite(value)
    }
    function canReturnToMap() {
        var z = sc && sc.ego ? sc.ego.z : null
        return sc && sc.source === "CARLA" && validNumber(sc.road_grade_deg)
            && validNumber(z) && validNumber(baseFlatZ)
            && Math.abs(sc.road_grade_deg) <= 0.15
            && Math.abs(z - baseFlatZ) <= 0.20
    }
    function updateSlopeMode() {
        // Read the new packet directly: dependent property bindings may update later.
        var z = sc && sc.ego ? sc.ego.z : null
        if (!sc || sc.source !== "CARLA" || !validNumber(sc.road_grade_deg) || !validNumber(z)) {
            mapReturnTimer.stop()
            return // Missing telemetry cannot prove a return to the original flat.
        }
        if (renderMode === "MAP_MODE") {
            if (Math.abs(sc.road_grade_deg) >= 0.40) {
                baseFlatZ = z // Capture once, only on MAP -> ROAD3D.
                renderMode = "ROAD3D_MODE"
            }
        }
        if (renderMode === "ROAD3D_MODE" && canReturnToMap()) {
            if (!mapReturnTimer.running) mapReturnTimer.start()
        } else {
            mapReturnTimer.stop()
        }
    }
    Timer {
        id: mapReturnTimer
        objectName: "mapReturnTimer"
        interval: 500
        repeat: false
        onTriggered: {
            if (w.renderMode === "ROAD3D_MODE" && w.canReturnToMap())
                w.renderMode = "MAP_MODE"
        }
    }
    onScChanged: updateSlopeMode()
    Component.onCompleted: updateSlopeMode()
    readonly property bool mapRoadOn: mapOn && !slopeMode
    readonly property bool mapLaneOn: mapRoadOn
    readonly property bool sceneRoadOn: !mapRoadOn // also handles missing/unready maps
    readonly property bool mapBoxesOn: mapOn && !slopeMode
    readonly property real vehicleHeightOffset: 0.0 // model assets already have ground Y=0
    readonly property color yellowLine: dark ? "#B08A2E" : "#F2B705"
    readonly property color boxColor: dark ? "#2C323A" : "#C8CCD3"

    readonly property bool dark: Theme.dark
    readonly property color bgColor: dark ? "#16191E" : "#E9EBEE"
    readonly property color roadColor: dark ? "#22272D" : "#D5D8DD"
    readonly property color lineColor: dark ? "#7E8792" : "#FFFFFF"
    readonly property color carColor: dark ? "#7C838C" : "#B9BDC3"
    readonly property color levelCol: Theme.levelColor(level)
    readonly property var lines: {
        var all = (sc && sc.lines3d) ? sc.lines3d : ((sc && sc.lines) ? sc.lines : [])
        // Flat map supplies all road markings; keep only the ego highlight overlay.
        return mapLaneOn ? (sc.on_road === false ? [] : all.filter(function(line) { return line.kind === "ego" })) : all
    }
    readonly property var vehicles: (sc && sc.vehicles) ? sc.vehicles : []
    readonly property bool driving: !!(sc && sc.source && sc.source !== "NONE") || (Theme.has(speed) && speed > 1)
    readonly property alias leadAnchor: leadAnchor
    property real sunBright: 0.35

    function debugVector(p) {
        if (!p) return "missing"
        return "(" + Number(p.x).toFixed(2) + "," + Number(p.y).toFixed(2) + "," + Number(p.z).toFixed(2) + ")"
    }
    function debugPoint(p) {
        return p ? "(" + p.map(function(n) { return Number(n).toFixed(2) }).join(",") + ")" : "missing"
    }
    function debugLine() {
        var d = sc.debug3d || {}
        var center = sc.center3d || []
        var p = null
        for (var i = 0; i < center.length; ++i)
            if (!p || Math.abs(center[i][0] - 25) < Math.abs(p[0] - 25)) p = center[i]
        var v = traffic.count > 0 ? traffic.objectAt(0) : null
        return "[3DDBG] stage=qml seq=" + sc.scene_seq + " frame=" + sc.scene_frame
            + " mode=" + (slopeMode ? "ROAD3D" : "MAP")
            + " egoZ=" + (validNumber(egoWorldZ) ? Number(egoWorldZ).toFixed(2) : "missing")
            + " baseFlatZ=" + (validNumber(baseFlatZ) ? Number(baseFlatZ).toFixed(2) : "missing")
            + " heightDelta=" + (validNumber(egoWorldZ) && validNumber(baseFlatZ) ? Number(egoWorldZ - baseFlatZ).toFixed(2) : "missing")
            + " egoPitch=" + (sc.ego ? sc.ego.pitch : "missing")
            + " roadGrade=" + (sc.road_grade_deg === null || sc.road_grade_deg === undefined ? "missing" : sc.road_grade_deg)
            + " roadMode=" + d.center_mode + " raw25=" + debugPoint(d.raw_center25)
            + " center25=" + debugPoint(p)
            + " roadQml=" + (p ? debugVector(Qt.vector3d(p[1], p[2], -p[0])) : "missing")
            + " rawVehicle=" + d.raw_vehicle_id + ":" + debugPoint(d.raw_vehicle_local)
            + " vehicle0=" + (v ? v.modelData.id + ":" + debugVector(Qt.vector3d(v.rx, v.ry, v.rz)) : "missing")
            + " final=" + (v ? debugVector(v.scenePosition) : "missing")
            + " rot=" + (v ? debugVector(v.eulerRotation) : "missing")
            + " vehicleMode=" + (v ? v.positionMode + "/" + v.rotationMode : "none")
            + " vehicles=" + vehicles.length + "/" + d.received_vehicles + " rawLocal=" + d.received_local
            + " slopeMode=" + Number(slopeMode) + " mapVisible=" + Number(mapOn)
            + " mapRoadVisible=" + Number(mapRoadOn) + " mapLaneVisible=" + Number(mapLaneOn)
            + " sceneRoad3DVisible=" + Number(sceneRoadOn) + " mapBoxes=" + Number(mapBoxesOn)
            + " root=" + debugVector(scenePosition) + "/" + debugVector(eulerRotation)
            + " ego=" + debugVector(ego.scenePosition) + "/" + debugVector(ego.eulerRotation)
    }

    // 점선 흐름: 텍스처 위치만 이동 (GPU), 형상은 장면이 바뀔 때만 다시 계산
    property real odo: 0
    Texture {
        id: dashTex
        source: "../assets/env/lane_dash.png"
        tilingModeVertical: Texture.Repeat
        positionV: -w.odo / 9.0
    }

    DirectionalLight {
        eulerRotation.x: -62
        eulerRotation.y: -28
        brightness: w.sunBright
        castsShadow: false          // 접지 그림자는 차량별 블롭으로 (라즈베리파이 GPU 부담 감소)
        shadowMapQuality: Light.ShadowMapQualityHigh
        shadowFactor: w.dark ? 60 : 55
        softShadowQuality: Light.PCF16
        pcfFactor: 4
        shadowMapFar: 120
        shadowBias: 8
    }

    // The flat background would occlude downhill geometry below Y=0.
    // 바닥
    Model {
        source: "#Rectangle"
        eulerRotation.x: -90
        visible: !w.slopeMode
        position: Qt.vector3d(0, -0.01, -60)
        scale: Qt.vector3d(12, 12, 1)
        receivesShadows: true
        materials: PrincipledMaterial { baseColor: w.bgColor; lighting: PrincipledMaterial.NoLighting }
    }

    // 도로면
    Model {
        objectName: "sceneRoad3D"
        visible: w.sceneRoadOn
        receivesShadows: true
        geometry: RibbonGeometry { left: w.sc ? w.sc.road_left3d || w.sc.road_left || [] : []; right: w.sc ? w.sc.road_right3d || w.sc.road_right || [] : []; lift: 0.0 }
        materials: PrincipledMaterial { baseColor: w.roadColor; lighting: PrincipledMaterial.NoLighting; cullMode: Material.NoCulling }
    }

    // ---------------- CARLA 실제 도로 (맵) ----------------
    // 형상은 앵커 좌표로 한 번만 만들고, 매 프레임 이 노드의 위치/회전만 바꿈 (map_xf)
    Texture { id: mapDashTex; source: "../assets/env/lane_dash.png"; tilingModeVertical: Texture.Repeat }
    Node {
        id: mapNode
        visible: w.mapOn
        readonly property var xf: w.sc && w.sc.map_xf ? w.sc.map_xf : ({ "x": 0, "y": 0, "rot": 0 })
        position: Qt.vector3d(xf.y, 0, -xf.x)
        eulerRotation.y: -xf.rot

        Model {
            visible: w.mapRoadOn     // 도로면 (모든 차선)
            receivesShadows: true
            geometry: RibbonGeometry { bands: w.carlaMap.bands || []; lift: 0.0 }
            materials: PrincipledMaterial { baseColor: w.roadColor; lighting: PrincipledMaterial.NoLighting; cullMode: Material.NoCulling }
        }
        Model {
            visible: w.mapLaneOn
            geometry: RibbonGeometry { strips: w.carlaMap.white || []; lift: 0.02 }
            materials: PrincipledMaterial { baseColor: w.lineColor; lighting: PrincipledMaterial.NoLighting; cullMode: Material.NoCulling }
        }
        Model {
            visible: w.mapLaneOn
            geometry: RibbonGeometry { strips: w.carlaMap.white_dash || []; dash: 3; gap: 6; lift: 0.02 }
            materials: PrincipledMaterial {
                baseColor: w.lineColor; baseColorMap: mapDashTex; alphaMode: PrincipledMaterial.Blend
                lighting: PrincipledMaterial.NoLighting; cullMode: Material.NoCulling
            }
        }
        Model {
            visible: w.mapLaneOn
            geometry: RibbonGeometry { strips: w.carlaMap.yellow || []; lift: 0.021 }
            materials: PrincipledMaterial { baseColor: w.yellowLine; lighting: PrincipledMaterial.NoLighting; cullMode: Material.NoCulling }
        }
        Model {
            visible: w.mapLaneOn
            geometry: RibbonGeometry { strips: w.carlaMap.yellow_dash || []; dash: 3; gap: 6; lift: 0.021 }
            materials: PrincipledMaterial {
                baseColor: w.yellowLine; baseColorMap: mapDashTex; alphaMode: PrincipledMaterial.Blend
                lighting: PrincipledMaterial.NoLighting; cullMode: Material.NoCulling
            }
        }
        // 건물·벽·펜스·기둥 (실제 위치·크기, 높이는 3 m 까지로 낮춰서 시야를 안 가리게)
        Repeater3D {
            model: w.carlaMap.boxes || []
            delegate: Model {
                required property var modelData
                visible: w.mapBoxesOn
                source: "#Cube"
                position: Qt.vector3d(modelData.y, modelData.h / 2, -modelData.x)
                eulerRotation.y: -modelData.yaw
                scale: Qt.vector3d(modelData.w / 100, modelData.h / 100, modelData.l / 100)
                materials: PrincipledMaterial { baseColor: w.boxColor; roughness: 0.9; specularAmount: 0.1 }
            }
        }
    }

    // 위험 영역 (자차 앞범퍼 ~ 앞차 뒤): L1 이상일 때 색
    Model {
        id: riskModel
        readonly property var rz: w.sc ? (w.sc.risk_zone3d || w.sc.risk_zone || []) : []
        visible: rz.length >= 4 && w.level >= 1
        receivesShadows: true
        geometry: RibbonGeometry {
            left: riskModel.rz.slice(0, riskModel.rz.length / 2)
            right: riskModel.rz.slice(riskModel.rz.length / 2).reverse()
            lift: 0.015
        }
        materials: PrincipledMaterial {
            baseColor: Qt.rgba(w.levelCol.r, w.levelCol.g, w.levelCol.b, 1)
            opacity: w.level >= 3 ? 0.6 : 0.42
            alphaMode: PrincipledMaterial.Blend
            lighting: PrincipledMaterial.NoLighting
            cullMode: Material.NoCulling
        }
    }

    // 차선
    Repeater3D {
        model: w.lines
        delegate: Model {
            required property var modelData
            receivesShadows: true
            geometry: RibbonGeometry {
                points: modelData.points
                width: modelData.kind === "ego" ? 0.16 : 0.13
                dash: modelData.kind === "dash" ? 3.0 : 0
                gap: modelData.kind === "dash" ? 6.0 : 0
                lift: 0.02
            }
            materials: PrincipledMaterial {
                baseColor: modelData.kind === "ego" && w.level >= 2 ? w.levelCol : w.lineColor
                baseColorMap: modelData.kind === "dash" ? dashTex : null
                alphaMode: modelData.kind === "dash" ? PrincipledMaterial.Blend : PrincipledMaterial.Opaque
                lighting: PrincipledMaterial.NoLighting
                cullMode: Material.NoCulling
            }
        }
    }

    // 주변 차량 / 보행자
    Repeater3D {
        id: traffic
        model: w.vehicles
        delegate: Vehicle3D {
            required property var modelData
            kind: modelData.model || "car"
            length: modelData.length || 4.6
            width: modelData.width || 1.9
            readonly property real rx: modelData.local_x !== undefined ? modelData.local_x : modelData.x
            readonly property real ry: modelData.local_y !== undefined ? modelData.local_y : modelData.y
            readonly property real rz: modelData.local_z !== undefined ? modelData.local_z : (modelData.z || 0)
            readonly property string positionMode: modelData.local_x !== undefined && modelData.local_y !== undefined && modelData.local_z !== undefined ? "local" : "legacy"
            readonly property string rotationMode: modelData.relative_pitch !== undefined ? "relative" : (modelData.local_pitch !== undefined ? "local-alias" : (positionMode === "local" ? "neutral" : "legacy"))
            position: Qt.vector3d(ry, rz + w.vehicleHeightOffset, -rx)
            // Local position packets must never reuse absolute pitch/roll.
            eulerRotation.x: modelData.relative_pitch !== undefined ? modelData.relative_pitch
                             : (modelData.local_pitch !== undefined ? modelData.local_pitch : (positionMode === "local" ? 0 : (modelData.pitch || 0)))
            eulerRotation.z: -(modelData.relative_roll !== undefined ? modelData.relative_roll
                              : (modelData.local_roll !== undefined ? modelData.local_roll : (positionMode === "local" ? 0 : (modelData.roll || 0))))
            eulerRotation.y: -(modelData.relative_yaw !== undefined ? modelData.relative_yaw
                              : (modelData.local_yaw !== undefined ? modelData.local_yaw : (modelData.yaw || 0)))
            visible: modelData.x < 160 && modelData.x > -40
            bodyColor: modelData.lead && w.level >= 1
                       ? Qt.tint(w.carColor, Qt.rgba(w.levelCol.r, w.levelCol.g, w.levelCol.b, w.level >= 3 ? 0.85 : 0.65))
                       : w.carColor
        }
    }

    // 자차
    EgoCar3D {
        id: ego
        objectName: "ego3d"
        position: Qt.vector3d(0, 0, 0)
        eulerRotation: Qt.vector3d(0, 0, 0)
        readonly property var e: w.sc && w.sc.ego ? w.sc.ego : ({})
        scale: Qt.vector3d(Math.max(0.8, Math.min(1.2, (e.width || 1.95) / 2.1)), 1, Math.max(0.8, Math.min(1.25, (e.length || 4.7) / 4.36)))
        braking: w.braking
        collided: !!(w.sc && w.sc.collided)
        paint: w.dark ? "#2F5BE0" : "#1846D0"
    }

    // 전방 차량 거리 라벨 위치용
    Node {
        id: leadAnchor
        readonly property var lead: {
            for (var i = 0; i < w.vehicles.length; i++) if (w.vehicles[i].lead) return w.vehicles[i]
            return null
        }
        position: !lead ? Qt.vector3d(0, 0, -1000)
                       : lead.local_x !== undefined && lead.local_y !== undefined && lead.local_z !== undefined
                         ? Qt.vector3d(lead.local_y, lead.local_z + 2.2, -lead.local_x)
                         : Qt.vector3d(lead.y, (lead.z || 0) + 2.2, -lead.x)
    }
}
