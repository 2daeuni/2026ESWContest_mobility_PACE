import QtQuick
import QtQuick3D

// 자차 3D 모델 (assets/models/ego_*.mesh, tools/build_vehicle_models.py 로 생성)
// 앞 = -Z, 지면 y = 0, 단위 m
Node {
    id: car
    property color paint: "#1846D0"
    property bool braking: false
    property bool collided: false        // CARLA 충돌 직후 1.5 초 빨갛게
    property bool lightsOn: true
    readonly property string base: "../assets/models/"

    PrincipledMaterial {
        id: paintMat
        baseColor: car.collided ? "#E02424" : car.paint
        metalness: 0.15
        roughness: 0.38
        clearcoatAmount: 0.2
        clearcoatRoughnessAmount: 0.04
        cullMode: Material.NoCulling
        specularAmount: 0.25
    }
    PrincipledMaterial { id: glassMat; baseColor: "#0D0F12"; metalness: 0.0; roughness: 0.18; specularAmount: 0.35; cullMode: Material.NoCulling }
    PrincipledMaterial { id: trimMat; baseColor: "#1E2126"; metalness: 0.2; roughness: 0.45 }
    PrincipledMaterial { id: tireMat; baseColor: "#17181A"; metalness: 0.0; roughness: 0.85 }
    PrincipledMaterial { id: rimMat; baseColor: "#B8BDC5"; metalness: 1.0; roughness: 0.28 }
    PrincipledMaterial {
        id: headMat
        baseColor: "#E8EEF5"
        emissiveFactor: car.lightsOn ? Qt.vector3d(0.9, 0.95, 1.0) : Qt.vector3d(0, 0, 0)
        lighting: PrincipledMaterial.NoLighting
    }
    PrincipledMaterial {
        id: brakeMat
        baseColor: car.braking ? "#FF2A2A" : "#5A0E12"
        emissiveFactor: car.braking ? Qt.vector3d(3.0, 0.15, 0.12) : Qt.vector3d(0.25, 0.0, 0.0)
        roughness: 0.2
    }
    PrincipledMaterial { id: signalMat; baseColor: "#6A3A0A"; roughness: 0.3 }

    // 바닥 접지 그림자 (부드러운 블롭, 가벼움)
    Model {
        source: "#Rectangle"
        eulerRotation.x: -90
        y: 0.025
        scale: Qt.vector3d(2.45 / 100, 4.9 / 100, 1)
        materials: PrincipledMaterial {
            baseColor: "black"
            baseColorMap: Texture { source: "../assets/env/contact_shadow.png" }
            alphaMode: PrincipledMaterial.Blend
            lighting: PrincipledMaterial.NoLighting
            opacity: 0.55
            depthDrawMode: Material.NeverDepthDraw
        }
    }

    Model { source: car.base + "ego_paint_mesh.mesh"; materials: [paintMat]; castsShadows: true }
    Model { source: car.base + "ego_accent_mesh.mesh"; materials: [paintMat]; castsShadows: true }
    Model { source: car.base + "ego_glass_mesh.mesh"; materials: [glassMat] }
    Model { source: car.base + "ego_trim_mesh.mesh"; materials: [trimMat] }
    Model { source: car.base + "ego_tire_mesh.mesh"; materials: [tireMat]; castsShadows: true }
    Model { source: car.base + "ego_rim_mesh.mesh"; materials: [rimMat] }
    Model { source: car.base + "ego_headlight_mesh.mesh"; materials: [headMat] }
    Model { source: car.base + "ego_brakelight_mesh.mesh"; materials: [brakeMat] }
    Model { source: car.base + "ego_signal_mesh.mesh"; materials: [signalMat] }
}
