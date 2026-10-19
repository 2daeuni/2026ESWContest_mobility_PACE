import QtQuick
import QtQuick3D

// 주변 차량 / 보행자 3D (테슬라 화면처럼 무광 회색 형상)
//   kind: car / truck / bus / bike / person   (hmi/scene.py model_category)
//   length / width: CARLA 바운딩박스 크기(m) → 모델 기준 크기에 맞춰 늘림
Node {
    id: v
    property string kind: "car"
    property real length: 4.6
    property real width: 1.9
    property color bodyColor: "#C9CDD3"
    property color darkColor: "#3B4047"
    property bool braking: false
    property real shadowOpacity: 0.5
    readonly property string base: "../assets/models/"

    // tools/build_vehicle_models.py 가 만든 assets/models/models.json 과 같은 값
    readonly property var dims: ({
        "car": [4.357, 2.542], "truck": [7.5, 2.4], "bus": [11.5, 2.5], "bike": [2.1, 0.8], "person": [0.5, 0.5]
    })
    readonly property var d: dims[kind] || dims["car"]
    readonly property real sx: kind === "person" ? 1.0 : Math.max(0.6, Math.min(1.6, width / d[1] * (kind === "car" ? 1.12 : 1.0)))
    readonly property real sz: kind === "person" ? 1.0 : Math.max(0.6, Math.min(1.6, length / d[0]))

    PrincipledMaterial { id: bodyMat; baseColor: v.bodyColor; metalness: 0.0; roughness: 0.6; specularAmount: 0.3; cullMode: Material.NoCulling }
    PrincipledMaterial { id: darkMat; baseColor: v.darkColor; metalness: 0.0; roughness: 0.5; specularAmount: 0.25; cullMode: Material.NoCulling }
    PrincipledMaterial { id: wheelMat; baseColor: "#2A2C30"; roughness: 0.8 }


    // 바닥 접지 그림자 (부드러운 블롭, 가벼움)
    Model {
        source: "#Rectangle"
        eulerRotation.x: -90
        y: 0.025
        scale: Qt.vector3d((v.kind === "person" ? 0.7 : v.width * 1.25) / 100, (v.kind === "person" ? 0.7 : v.length * 1.12) / 100, 1)
        materials: PrincipledMaterial {
            baseColor: "black"
            baseColorMap: Texture { source: "../assets/env/contact_shadow.png" }
            alphaMode: PrincipledMaterial.Blend
            lighting: PrincipledMaterial.NoLighting
            opacity: v.shadowOpacity
            depthDrawMode: Material.NeverDepthDraw
        }
    }

    Node {
        scale: Qt.vector3d(v.sx, 1, v.sz)
        Model { source: v.base + v.kind + "_body_mesh.mesh"; materials: [bodyMat]; castsShadows: true }
        Model { visible: v.kind !== "person" && v.kind !== "bike"; source: visible ? v.base + v.kind + "_dark_mesh.mesh" : ""; materials: [darkMat] }
        Model { visible: v.kind !== "person"; source: visible ? v.base + v.kind + "_wheel_mesh.mesh" : ""; materials: [wheelMat] }
    }
}
