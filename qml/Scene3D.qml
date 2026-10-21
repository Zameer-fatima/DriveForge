import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import DriveForge

// 3D world: textured 3-lane road, scenery recycled around the car, follow camera.
// Sim (x right, y forward) maps to scene (x, 0, -y); heading psi (clockwise) maps to eulerRotation.y = -psi.
View3D {
    id: view
    required property SimulationController sim

    // Graphics quality: 0 low, 1 medium, 2 high (cycled with F4, set at start with --quality)
    property int quality: startupQuality

    // ---- road geometry (matches driveforge::Road) ----
    readonly property real laneWidth: 3.6
    readonly property real roadHalf: laneWidth * 1.5

    // The road is built from tiles that are recycled as the car drives, so textures are fixed to the world.
    readonly property real tileLength: 24
    readonly property int tileCount: 30
    readonly property int tileBase: Math.floor(sim.posY / tileLength)
    readonly property real treeStep: 31
    readonly property int treeBase: Math.floor(sim.posY / treeStep)

    // ---- camera ----
    // Parking view: while parking assist is on, the camera rises and looks from the left towards the bays on the right.
    readonly property bool parkingView: sim.parkingEnabled
    property real parkBlend: parkingView ? 1 : 0
    Behavior on parkBlend { NumberAnimation { duration: 800; easing.type: Easing.InOutQuad } }
    function lerp(a, b) { return a + (b - a) * parkBlend }
    property real cameraYaw: parkingView ? 0 : -sim.headingDeg
    Behavior on cameraYaw { NumberAnimation { duration: 350; easing.type: Easing.OutQuad } }

    // Mouse camera: drag to orbit the car, wheel to zoom, double-click (or Home) to reset. Applied on top of any preset.
    property real orbitYaw: 0       // degrees around the car
    property real orbitPitch: 0     // degrees of extra elevation (negative = lower)
    property real orbitZoom: 1.0    // distance multiplier
    function resetCamera() { orbitYaw = 0; orbitPitch = 0; orbitZoom = 1.0 }

    // Screenshot presets (--camera): chase (default), front, rear, side, top, closeup
    readonly property string preset: startupCamera
    readonly property var presets: ({
        "front":   { yaw: 150, x: 0, y: 1.5, z: 6.2, pitch: -9, fov: 50 },
        "rear":    { yaw: 28,  x: 0, y: 1.6, z: 6.6, pitch: -9, fov: 50 },
        "side":    { yaw: 90,  x: 0, y: 1.1, z: 6.8, pitch: -5, fov: 45 },
        "top":     { yaw: 0,   x: 0, y: 17,  z: 3,   pitch: -80, fov: 55 },
        "closeup": { yaw: 40,  x: 0, y: 1.0, z: 4.2, pitch: -4, fov: 45 }
    })
    readonly property bool usePreset: presets[preset] !== undefined
    readonly property var cam: usePreset ? presets[preset] : null

    // ---- look: HDR sky for lighting and reflections, depth haze, bloom, soft ambient occlusion ----
    environment: ExtendedSceneEnvironment {
        backgroundMode: SceneEnvironment.SkyBox
        lightProbe: Texture { source: "../assets/sky/kloofendal_partly_cloudy_1k.hdr" }
        probeExposure: 0.85
        skyboxBlurAmount: 0.0
        antialiasingMode: view.quality >= 1 ? SceneEnvironment.MSAA : SceneEnvironment.NoAA
        antialiasingQuality: view.quality >= 2 ? SceneEnvironment.High : SceneEnvironment.Medium
        fxaaEnabled: view.quality === 0
        tonemapMode: SceneEnvironment.TonemapModeFilmic
        exposure: 0.8
        aoEnabled: view.quality >= 2
        aoStrength: 35
        aoDistance: 1.2
        aoSoftness: 40
        glowEnabled: view.quality >= 1
        glowStrength: 0.35
        glowIntensity: 0.8
        glowBloom: 0.12
        glowHDRMaximumValue: 8.0
        glowHDRScale: 2.0
        fog: Fog {
            enabled: true
            color: "#b4c7da"
            depthEnabled: true
            depthNear: 140
            depthFar: 1100
            depthCurve: 1.4
        }
    }

    DirectionalLight {
        eulerRotation: Qt.vector3d(-38, 28, 0)
        brightness: 1.5
        color: "#fff1dc"
        castsShadow: true
        shadowMapQuality: view.quality >= 2 ? Light.ShadowMapQualityVeryHigh : view.quality === 1 ? Light.ShadowMapQualityHigh : Light.ShadowMapQualityMedium
        shadowMapFar: 160
        shadowBias: 0.03
        shadowFactor: 75
        shadowFilter: view.quality >= 2 ? 8 : view.quality === 1 ? 5 : 2
    }

    // ---- materials ----
    PrincipledMaterial {
        id: asphaltMat
        roughness: 1.0
        baseColor: "#9a9a9a"
        baseColorMap: Texture { id: aC; source: "../assets/textures/asphalt_color.jpg"; tilingModeHorizontal: Texture.Repeat; tilingModeVertical: Texture.Repeat; scaleU: 10.8 / 4; scaleV: view.tileLength / 4; generateMipmaps: true; mipFilter: Texture.Linear }
        normalMap: Texture { source: "../assets/textures/asphalt_normal.jpg"; tilingModeHorizontal: Texture.Repeat; tilingModeVertical: Texture.Repeat; scaleU: 10.8 / 4; scaleV: view.tileLength / 4; generateMipmaps: true; mipFilter: Texture.Linear }
        normalStrength: 0.9
        roughnessMap: Texture { source: "../assets/textures/asphalt_roughness.jpg"; tilingModeHorizontal: Texture.Repeat; tilingModeVertical: Texture.Repeat; scaleU: 10.8 / 4; scaleV: view.tileLength / 4; generateMipmaps: true; mipFilter: Texture.Linear }
        roughnessChannel: Material.R
    }
    PrincipledMaterial {
        id: shoulderMat
        roughness: 1.0
        baseColor: "#8f8e88"
        baseColorMap: Texture { source: "../assets/textures/asphalt_color.jpg"; tilingModeHorizontal: Texture.Repeat; tilingModeVertical: Texture.Repeat; scaleU: 2.5 / 4; scaleV: view.tileLength / 4; generateMipmaps: true; mipFilter: Texture.Linear }
        normalMap: Texture { source: "../assets/textures/asphalt_normal.jpg"; tilingModeHorizontal: Texture.Repeat; tilingModeVertical: Texture.Repeat; scaleU: 2.5 / 4; scaleV: view.tileLength / 4; generateMipmaps: true; mipFilter: Texture.Linear }
        roughnessMap: Texture { source: "../assets/textures/asphalt_roughness.jpg"; tilingModeHorizontal: Texture.Repeat; tilingModeVertical: Texture.Repeat; scaleU: 2.5 / 4; scaleV: view.tileLength / 4; generateMipmaps: true; mipFilter: Texture.Linear }
        roughnessChannel: Material.R
    }
    PrincipledMaterial {
        id: grassMat
        roughness: 1.0
        baseColor: "#cfe3b0"
        baseColorMap: Texture { source: "../assets/textures/grass_color.jpg"; tilingModeHorizontal: Texture.Repeat; tilingModeVertical: Texture.Repeat; scaleU: 60 / 6; scaleV: view.tileLength / 6; generateMipmaps: true; mipFilter: Texture.Linear }
        normalMap: Texture { source: "../assets/textures/grass_normal.jpg"; tilingModeHorizontal: Texture.Repeat; tilingModeVertical: Texture.Repeat; scaleU: 60 / 6; scaleV: view.tileLength / 6; generateMipmaps: true; mipFilter: Texture.Linear }
        roughnessMap: Texture { source: "../assets/textures/grass_roughness.jpg"; tilingModeHorizontal: Texture.Repeat; tilingModeVertical: Texture.Repeat; scaleU: 60 / 6; scaleV: view.tileLength / 6; generateMipmaps: true; mipFilter: Texture.Linear }
        roughnessChannel: Material.R
    }
    PrincipledMaterial { id: farGroundMat; baseColor: "#566f3d"; roughness: 1 }
    PrincipledMaterial { id: hillMat; baseColor: "#4d6a45"; roughness: 1 }
    PrincipledMaterial { id: paintMat; baseColor: "#f4f4f2"; roughness: 0.55 }
    PrincipledMaterial { id: railMat; baseColor: "#aeb4ba"; metalness: 0.85; roughness: 0.4 }
    PrincipledMaterial { id: postMat; baseColor: "#8a8f94"; metalness: 0.6; roughness: 0.6 }
    PrincipledMaterial { id: signPoleMat; baseColor: "#9aa0a6"; metalness: 0.8; roughness: 0.5 }
    PrincipledMaterial { id: signFaceMat; roughness: 0.4; baseColor: "white" }

    // ---- everything below follows the car along the road ----
    Node {
        position: Qt.vector3d(0, 0, -view.sim.posY)

        // Far ground and hills: they stay put relative to the car, like things at infinity, and fade into the haze.
        Model {
            source: "#Rectangle"; materials: farGroundMat
            eulerRotation.x: -90
            position: Qt.vector3d(0, -0.12, 0)
            scale: Qt.vector3d(80, 80, 1)
            receivesShadows: false
        }
        Repeater3D {
            model: 9
            delegate: Model {
                required property int index
                source: "#Sphere"; materials: hillMat
                readonly property real a: (index - 4) * 0.5
                position: Qt.vector3d(Math.sin(a) * 1500 + (index % 2 ? 120 : -80), 0, -Math.cos(a) * 1500)
                scale: Qt.vector3d(9 + (index * 7 % 5), 1.7 + (index * 3 % 4) * 0.5, 7 + (index * 5 % 4))
            }
        }
    }

    // Road tiles
    Repeater3D {
        model: view.tileCount
        delegate: Node {
            id: tile
            required property int index
            readonly property int tileId: view.tileBase - 3 + index
            position: Qt.vector3d(0, 0, -(tile.tileId * view.tileLength + view.tileLength / 2))

            // surfaces (a #Rectangle is 100 x 100 units in the XY plane; rotate it to lie on the ground)
            Model { source: "#Rectangle"; materials: asphaltMat; receivesShadows: true; eulerRotation.x: -90
                    position: Qt.vector3d(0, -0.02, 0); scale: Qt.vector3d(view.roadHalf * 2 / 100, view.tileLength / 100, 1) }
            Model { source: "#Rectangle"; materials: shoulderMat; receivesShadows: true; eulerRotation.x: -90
                    position: Qt.vector3d(-(view.roadHalf + 1.25), -0.03, 0); scale: Qt.vector3d(2.5 / 100, view.tileLength / 100, 1) }
            Model { source: "#Rectangle"; materials: shoulderMat; receivesShadows: true; eulerRotation.x: -90
                    position: Qt.vector3d(view.roadHalf + 1.25, -0.03, 0); scale: Qt.vector3d(2.5 / 100, view.tileLength / 100, 1) }
            Model { source: "#Rectangle"; materials: grassMat; receivesShadows: true; eulerRotation.x: -90
                    position: Qt.vector3d(-(view.roadHalf + 2.5 + 30), -0.06, 0); scale: Qt.vector3d(0.6, view.tileLength / 100, 1) }
            Model { source: "#Rectangle"; materials: grassMat; receivesShadows: true; eulerRotation.x: -90
                    position: Qt.vector3d(view.roadHalf + 2.5 + 30, -0.06, 0); scale: Qt.vector3d(0.6, view.tileLength / 100, 1) }

            // edge lines and dashed lane dividers
            Repeater3D {
                model: 2
                delegate: Model {
                    required property int index
                    source: "#Rectangle"; materials: paintMat; eulerRotation.x: -90
                    position: Qt.vector3d((index === 0 ? -1 : 1) * (view.roadHalf - 0.30), -0.015, 0)
                    scale: Qt.vector3d(0.16 / 100, view.tileLength / 100, 1)
                }
            }
            Repeater3D {
                model: 4
                delegate: Model {
                    required property int index
                    source: "#Rectangle"; materials: paintMat; eulerRotation.x: -90
                    position: Qt.vector3d((index % 2 ? 1 : -1) * view.laneWidth / 2, -0.015, index < 2 ? -10.5 : 1.5)
                    scale: Qt.vector3d(0.15 / 100, 3 / 100, 1)
                }
            }

            // guardrails on both sides: W-beam rail, posts every 6 m
            Repeater3D {
                model: 2
                delegate: Node {
                    required property int index
                    readonly property real side: index === 0 ? -1 : 1
                    position: Qt.vector3d(side * (view.roadHalf + 2.9), 0, 0)
                    Model { source: "#Cube"; materials: railMat; castsShadows: true
                            position: Qt.vector3d(0, 0.62, 0); scale: Qt.vector3d(0.05 / 100, 0.30 / 100, view.tileLength / 100) }
                    Model { source: "#Cube"; materials: railMat; position: Qt.vector3d(0, 0.40, 0); scale: Qt.vector3d(0.04 / 100, 0.12 / 100, view.tileLength / 100) }
                    Repeater3D {
                        model: 4
                        delegate: Model {
                            required property int index
                            source: "#Cube"; materials: postMat; castsShadows: true
                            position: Qt.vector3d(0.07, 0.40, -9 + index * 6)
                            scale: Qt.vector3d(0.09 / 100, 0.80 / 100, 0.12 / 100)
                        }
                    }
                }
            }

            // a speed-limit sign on the right every 12 tiles
            Node {
                visible: ((tile.tileId % 12) + 12) % 12 === 0
                position: Qt.vector3d(view.roadHalf + 4.6, 0, 0)
                Model { source: "#Cylinder"; materials: signPoleMat; castsShadows: true
                        position: Qt.vector3d(0, 1.5, 0); scale: Qt.vector3d(0.08 / 100, 3.0 / 100, 0.08 / 100) }
                Model {
                    source: "#Cylinder"; castsShadows: true
                    position: Qt.vector3d(-0.04, 2.7, 0); eulerRotation.z: 90
                    scale: Qt.vector3d(0.9 / 100, 0.04 / 100, 0.9 / 100)
                    materials: [ PrincipledMaterial { baseColor: "#d62020"; roughness: 0.45 } ]
                }
                Model {
                    source: "#Cylinder"
                    position: Qt.vector3d(-0.07, 2.7, 0); eulerRotation.z: 90
                    scale: Qt.vector3d(0.66 / 100, 0.04 / 100, 0.66 / 100)
                    materials: [ PrincipledMaterial { baseColor: "#f4f4f4"; roughness: 0.4 } ]
                }
                Model {   // the number: a 3D text-like pair of bars would be crude, so draw it to a texture
                    source: "#Rectangle"
                    position: Qt.vector3d(-0.1, 2.7, 0); eulerRotation.y: -90
                    scale: Qt.vector3d(0.6 / 100, 0.6 / 100, 1)
                    materials: [ PrincipledMaterial {
                        lighting: PrincipledMaterial.NoLighting
                        baseColorMap: Texture { sourceItem: Item {
                            width: 128; height: 128
                            Text { anchors.centerIn: parent; text: "100"; font.pixelSize: 62; font.bold: true; color: "#101010" }
                        } }
                        alphaMode: PrincipledMaterial.Blend
                    } ]
                }
            }
        }
    }

    // Trees along both sides, a mix of pines and broadleaf with varied size and colour
    Repeater3D {
        model: 2 * 26
        delegate: Tree3D {
            id: treeItem
            required property int index
            readonly property int side: index % 2
            readonly property int slot: Math.floor(index / 2)
            readonly property int worldSlot: view.treeBase - 8 + slot
            readonly property int h: Math.abs((worldSlot * 7919 + side * 104729) % 1000)
            species: h % 3 === 0 ? 1 : 0
            size: 0.85 + (h % 7) * 0.07
            tint: ((h % 11) - 5) / 5
            position: Qt.vector3d((side === 0 ? -1 : 1) * (view.roadHalf + 9 + (h % 17)), 0, -worldSlot * view.treeStep + (h % 13) - 6)
            eulerRotation.y: h % 360
        }
    }

    // ---- bays, lot floor and the planned parking path ----
    Model {
        visible: view.sim.bays.hasBays
        source: "#Rectangle"; eulerRotation.x: -90
        materials: shoulderMat
        position: Qt.vector3d(view.sim.bays.lotX - 1.5, -0.04, -(view.sim.bays.lotMinY + view.sim.bays.lotMaxY) / 2)
        scale: Qt.vector3d((view.sim.bays.lotDepth + 6.5) / 100, (view.sim.bays.lotMaxY - view.sim.bays.lotMinY + 4) / 100, 1)
        receivesShadows: true
    }
    Repeater3D {
        model: view.sim.bays
        delegate: Node {
            id: bay
            required property int state  // 0 other, 1 free, 2 target
            required property real posX
            required property real posY
            required property real width
            required property real length
            position: Qt.vector3d(posX, 0, -posY)
            readonly property color tone: state === 2 ? "#3aa0ff" : state === 1 ? "#3ddc84" : "#e6e6e6"
            PrincipledMaterial { id: lineMat; baseColor: bay.tone; emissiveFactor: bay.state > 0 ? Qt.vector3d(0.15, 0.3, 0.2) : Qt.vector3d(0, 0, 0); roughness: 0.7 }
            Model {   // fill, only for free / target bays
                visible: bay.state > 0
                source: "#Rectangle"; eulerRotation.x: -90
                materials: PrincipledMaterial { baseColor: Qt.rgba(bay.tone.r, bay.tone.g, bay.tone.b, 0.30); alphaMode: PrincipledMaterial.Blend; roughness: 1 }
                position: Qt.vector3d(0, -0.025, 0)
                scale: Qt.vector3d(bay.length / 100, bay.width / 100, 1)
            }
            Model { source: "#Cube"; materials: lineMat; position: Qt.vector3d(0, -0.02, -bay.width / 2); scale: Qt.vector3d(bay.length / 100, 0.001, 0.12 / 100) }
            Model { source: "#Cube"; materials: lineMat; position: Qt.vector3d(0, -0.02, bay.width / 2); scale: Qt.vector3d(bay.length / 100, 0.001, 0.12 / 100) }
            Model { source: "#Cube"; materials: lineMat; position: Qt.vector3d(bay.length / 2, -0.02, 0); scale: Qt.vector3d(0.12 / 100, 0.001, bay.width / 100) }
            Model {   // wheel stop at the back of the bay
                source: "#Cube"; castsShadows: true
                materials: PrincipledMaterial { baseColor: "#9d9d98"; roughness: 0.9 }
                position: Qt.vector3d(bay.length / 2 - 0.5, 0.06, 0); scale: Qt.vector3d(0.15 / 100, 0.12 / 100, 1.7 / 100)
            }
        }
    }
    PrincipledMaterial { id: pathMarkerMat; baseColor: "#27c9ff"; emissiveFactor: Qt.vector3d(0.3, 1.4, 2.0); roughness: 0.5 }
    Repeater3D {
        model: view.sim.parkingPath
        delegate: Model {
            required property var modelData
            source: "#Sphere"
            materials: pathMarkerMat
            position: Qt.vector3d(modelData.x, 0.12, -modelData.y)
            scale: Qt.vector3d(0.18 / 100, 0.06 / 100, 0.18 / 100)
        }
    }

    // ---- other actors: vehicles, barriers, pedestrians ----
    Repeater3D {
        model: view.sim.objects
        delegate: Node {
            id: obj
            required property int objId
            required property int kind  // 0 vehicle, 1 obstacle, 2 pedestrian
            required property real posX
            required property real posY
            required property real width
            required property real length
            required property real velX
            required property real velY
            required property bool moving
            position: Qt.vector3d(posX, 0, -posY)

            readonly property var palette: ["#b32d2d", "#e6e6e6", "#2c2f33", "#8a9097", "#2e6b3a", "#c9a227", "#6b3fa0", "#1c5fa8"]
            readonly property var jackets: ["#c0392b", "#2980b9", "#27ae60", "#8e44ad", "#d35400", "#34495e"]

            // Vehicles: a parked car lies across its bay (wider than long), a moving one points along the road.
            VehicleBody {
                visible: obj.kind === 0
                readonly property bool across: obj.width > obj.length
                eulerRotation.y: across ? 90 : 0
                length: Math.max(obj.width, obj.length)
                width: Math.min(obj.width, obj.length)
                style: obj.objId % 3 === 2 ? 1 : (obj.objId % 7 === 5 ? 2 : 0)
                paint: obj.palette[obj.objId % obj.palette.length]
                wheelSpinDeg: across ? 0 : -(obj.posY / wheelRadius) * 57.2958
                brakeLevel: 0
                detailed: view.quality >= 2
            }

            // Obstacle: a plastic road barrier in orange and white with a flashing lamp
            Node {
                visible: obj.kind === 1
                PrincipledMaterial { id: orange; baseColor: "#f26a10"; roughness: 0.55 }
                PrincipledMaterial { id: white; baseColor: "#f2f2f2"; roughness: 0.55 }
                Repeater3D {
                    model: 4
                    delegate: Model {
                        required property int index
                        source: "#Cube"; castsShadows: true
                        materials: index % 2 ? white : orange
                        position: Qt.vector3d(-obj.width / 2 + obj.width / 8 + index * obj.width / 4, 0.42, 0)
                        scale: Qt.vector3d(obj.width / 4 / 100, 0.84 / 100, obj.length / 100)
                    }
                }
                Model { source: "#Cube"; materials: white; castsShadows: true; position: Qt.vector3d(0, 0.1, 0); scale: Qt.vector3d((obj.width + 0.1) / 100, 0.2 / 100, (obj.length + 0.2) / 100) }
                Model {
                    source: "#Sphere"
                    position: Qt.vector3d(0, 0.95, 0); scale: Qt.vector3d(0.16 / 100, 0.16 / 100, 0.16 / 100)
                    materials: PrincipledMaterial { baseColor: "#ffb000"; emissiveFactor: lamp.on ? Qt.vector3d(4, 2.2, 0) : Qt.vector3d(0.3, 0.15, 0) }
                    Item { id: lamp; property bool on: true; Timer { running: obj.kind === 1; interval: 450; repeat: true; onTriggered: lamp.on = !lamp.on } }
                }
            }

            // Pedestrian: walks in the direction it is moving, swinging arms and legs
            Pedestrian3D {
                visible: obj.kind === 2
                walking: obj.moving
                phase: posX
                facing: obj.velX < 0 ? 90 : -90
                jacket: obj.jackets[obj.objId % obj.jackets.length]
            }
        }
    }

    // ---- the player's car ----
    Car {
        position: Qt.vector3d(view.sim.posX, 0, -view.sim.posY)
        eulerRotation.y: -view.sim.headingDeg
        steerDeg: view.sim.steeringDeg
        wheelSpinDeg: view.sim.wheelSpinDeg
        brakeLevel: view.sim.brake
        accel: view.sim.acceleration
        lateral: view.sim.lateralAcceleration
        reverse: view.sim.reverse
        laneChangeDir: view.sim.laneChangeDir
        emergency: view.sim.aebBraking
    }

    // ---- follow camera: the rig sits on the car and follows its heading smoothly. The arm swings the camera round the car
    // (parking view, presets and the mouse), the pitch node lifts or lowers it, and the camera sits at a distance on that arm. ----
    Node {
        position: Qt.vector3d(view.sim.posX, 0, -view.sim.posY)
        eulerRotation.y: view.usePreset ? -view.sim.headingDeg : view.cameraYaw
        Node {
            eulerRotation.y: (view.usePreset ? view.cam.yaw : view.lerp(0, -38)) + view.orbitYaw
            Node {
                eulerRotation.x: -view.orbitPitch
                PerspectiveCamera {
                    id: followCamera
                    readonly property vector3d basePosition: view.usePreset ? Qt.vector3d(view.cam.x, view.cam.y, view.cam.z)
                                                                            : Qt.vector3d(0, view.lerp(3.2, 15), view.lerp(8.5, 16))
                    position: basePosition.times(view.orbitZoom)
                    eulerRotation.x: view.usePreset ? view.cam.pitch : view.lerp(-10, -38)
                    fieldOfView: view.usePreset ? view.cam.fov : 58
                    clipNear: 0.2
                    clipFar: 3000
                }
            }
        }
    }
    camera: followCamera

    MouseArea {
        id: orbitMouse
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
        cursorShape: pressed ? Qt.ClosedHandCursor : Qt.OpenHandCursor
        property real lastX: 0
        property real lastY: 0
        onPressed: mouse => { lastX = mouse.x; lastY = mouse.y }
        onPositionChanged: mouse => {
            if (!pressed) return
            view.orbitYaw -= (mouse.x - lastX) * 0.35                                   // drag right: the camera swings left round the car
            view.orbitPitch = Math.max(-14, Math.min(70, view.orbitPitch + (mouse.y - lastY) * 0.30))  // drag down: look from higher up
            lastX = mouse.x
            lastY = mouse.y
        }
        onWheel: wheel => { view.orbitZoom = Math.max(0.35, Math.min(3.0, view.orbitZoom * (wheel.angleDelta.y > 0 ? 0.9 : 1.1))) }
        onDoubleClicked: view.resetCamera()
    }
}
