import QtQuick
import QtQuick3D
import DriveForge

// Procedural car. Local frame: forward = -Z, right = +X, origin on the ground at the car centre. Units are metres.
// The body is lofted in C++ (CarBodyGeometry); everything else here is primitives. Used for the ego car and all other vehicles.
Node {
    id: car

    property int style: 0              // 0 sedan, 1 SUV, 2 van
    property real length: 4.5
    property real width: 1.85
    property color paint: "#1f4fa8"
    property real steerDeg: 0          // positive = right
    property real wheelSpinDeg: 0
    property real brakeLevel: 0
    property bool reverseLights: false
    property int blinker: 0            // -1 left, +1 right
    property bool hazard: false
    property real pitchDeg: 0          // body motion on the suspension
    property real rollDeg: 0
    property bool castsShadows: true
    property bool detailed: true       // spokes, door lines, handles, mirrors, grille (skipped for background cars at lower quality)

    readonly property real wheelRadius: style === 0 ? 0.33 : style === 1 ? 0.37 : 0.38
    readonly property real axleZ: 0.30 * length
    readonly property real trackHalf: width / 2 - 0.10
    // Lamp positions sit on the surface of the lofted body, per style: [sedan, SUV, van]
    readonly property var noseY: [0.66, 0.84, 0.90]    // headlight height
    readonly property var headZ: [0.30, 0.34, 0.36]    // headlight distance behind the front tip
    readonly property var tailY: [0.80, 0.98, 1.00]    // tail light height
    readonly property var tailZ: [0.42, 0.34, 0.30]    // tail light distance in front of the rear tip
    readonly property var beltY: [0.96, 1.10, 1.10]    // top of the lower body
    property bool blinkOn: true

    function sc(w, h, d) { return Qt.vector3d(w / 100, h / 100, d / 100) }  // #Cube / #Sphere / #Cylinder are 100 units wide

    Timer { running: car.blinker !== 0 || car.hazard; interval: 380; repeat: true; onTriggered: car.blinkOn = !car.blinkOn }

    PrincipledMaterial {
        id: paintMat
        baseColor: car.paint
        metalness: 0.35
        roughness: 0.38
        clearcoatAmount: 1.0
        clearcoatRoughnessAmount: 0.03
    }
    PrincipledMaterial { id: glassMat; baseColor: "#0a1016"; metalness: 0.25; roughness: 0.04 }
    PrincipledMaterial { id: trimMat; baseColor: "#17191c"; roughness: 0.75 }
    PrincipledMaterial { id: chromeMat; baseColor: "#d4d8dc"; metalness: 1.0; roughness: 0.18 }
    PrincipledMaterial { id: tyreMat; baseColor: "#0e0e0f"; roughness: 0.92 }
    PrincipledMaterial { id: rimMat; baseColor: "#c9ced3"; metalness: 1.0; roughness: 0.22 }
    PrincipledMaterial { id: discMat; baseColor: "#55575a"; metalness: 0.9; roughness: 0.45 }
    PrincipledMaterial { id: plateMat; baseColor: "#f3f1e6"; roughness: 0.6 }
    PrincipledMaterial { id: headMat; baseColor: "#c9d2dc"; metalness: 0.4; roughness: 0.08; emissiveFactor: Qt.vector3d(0.55, 0.55, 0.62) }
    PrincipledMaterial { id: drlMat; baseColor: "#ffffff"; emissiveFactor: Qt.vector3d(1.6, 1.6, 1.7) }
    PrincipledMaterial {
        id: tailMat
        baseColor: "#6e0a0a"
        roughness: 0.15
        emissiveFactor: car.brakeLevel > 0.05 ? Qt.vector3d(2.2, 0.05, 0.03) : Qt.vector3d(0.45, 0.015, 0.01)
    }
    PrincipledMaterial { id: reverseMat; baseColor: "#8d9299"; metalness: 0.3; roughness: 0.15; emissiveFactor: car.reverseLights ? Qt.vector3d(3, 3, 3) : Qt.vector3d(0, 0, 0) }
    PrincipledMaterial { id: amberOn; baseColor: "#a35c00"; emissiveFactor: Qt.vector3d(4.0, 1.8, 0.0) }
    PrincipledMaterial { id: amberOff; baseColor: "#9a5a10"; roughness: 0.2; metalness: 0.2 }

    // ---- body on its suspension: pivots about the axle height ----
    Node {
        y: 0.30
        eulerRotation: Qt.vector3d(car.pitchDeg, 0, car.rollDeg)
        Node {
            y: -0.30

            Model {
                castsShadows: car.castsShadows; receivesShadows: true
                geometry: CarBodyGeometry { part: CarBodyGeometry.LowerBody; style: car.style; length: car.length; width: car.width }
                materials: [paintMat, trimMat, headMat, tailMat,
                            (car.blinkOn && (car.hazard || car.blinker === -1)) ? amberOn : amberOff,
                            (car.blinkOn && (car.hazard || car.blinker === 1)) ? amberOn : amberOff,
                            reverseMat]
            }
            Model {
                castsShadows: car.castsShadows
                geometry: CarBodyGeometry { part: CarBodyGeometry.Cabin; style: car.style; length: car.length; width: car.width }
                materials: [paintMat, glassMat]
            }

            // Mirrors, door lines and handles (the lamps are part of the body mesh itself) and the grille
            Repeater3D {
                visible: car.detailed
                model: car.detailed ? [-1, 1] : []
                delegate: Node {
                    required property int modelData
                    // Side mirrors
                    Model {
                        source: "#Sphere"; materials: paintMat; castsShadows: car.castsShadows
                        position: Qt.vector3d(modelData * (car.width / 2 - 0.02), car.beltY[car.style] + 0.07, -car.length * 0.13)
                        scale: car.sc(0.12, 0.09, 0.17)
                    }
                    Model {
                        source: "#Cube"; materials: trimMat
                        position: Qt.vector3d(modelData * (car.width / 2 - 0.10), car.beltY[car.style] + 0.03, -car.length * 0.13)
                        scale: car.sc(0.10, 0.03, 0.04)
                    }
                    // Door shut lines and handles
                    Model {
                        source: "#Cube"; materials: trimMat
                        position: Qt.vector3d(modelData * (car.width / 2 - 0.012), car.noseY[car.style] - 0.06, -car.length * 0.045)
                        scale: car.sc(0.012, 0.50, 0.012)
                    }
                    Model {
                        source: "#Cube"; materials: trimMat
                        position: Qt.vector3d(modelData * (car.width / 2 - 0.012), car.noseY[car.style] - 0.06, car.length * 0.17)
                        scale: car.sc(0.012, 0.50, 0.012)
                    }
                    Model {
                        source: "#Cube"; materials: chromeMat
                        position: Qt.vector3d(modelData * (car.width / 2 - 0.005), car.beltY[car.style] - 0.12, car.length * 0.02)
                        scale: car.sc(0.012, 0.025, 0.12)
                    }
                }
            }
            Model {   // grille
                visible: car.detailed
                source: "#Cube"; materials: trimMat
                position: Qt.vector3d(0, car.noseY[car.style] - 0.12, -car.length / 2 + 0.07)
                scale: car.sc(car.width * 0.44, 0.12, 0.05)
            }
            Model {   // chrome bar across the grille
                visible: car.detailed
                source: "#Cube"; materials: chromeMat
                position: Qt.vector3d(0, car.noseY[car.style] - 0.12, -car.length / 2 + 0.045)
                scale: car.sc(car.width * 0.38, 0.014, 0.012)
            }
            Model {   // front and rear number plates
                source: "#Cube"; materials: plateMat
                position: Qt.vector3d(0, car.noseY[car.style] - 0.26, -car.length / 2 + 0.115)
                scale: car.sc(0.52, 0.11, 0.008)
            }
            Model {
                source: "#Cube"; materials: plateMat
                position: Qt.vector3d(0, car.tailY[car.style] - 0.33, car.length / 2 - 0.12)
                scale: car.sc(0.52, 0.11, 0.008)
            }
            Model {   // dark underbody so the gap below the car is not see-through
                source: "#Cube"; materials: trimMat
                position: Qt.vector3d(0, 0.33, 0)
                scale: car.sc(car.width * 0.66, 0.04, car.length * 0.74)
            }
        }
    }

    // ---- wheels (not part of the body motion) ----
    component Wheel: Node {
        id: wheel
        property bool steered: false
        property real side: 1    // +1 right, -1 left
        eulerRotation.y: steered ? -car.steerDeg : 0
        Node {
            eulerRotation.x: -car.wheelSpinDeg
            Model {   // tyre
                source: "#Cylinder"; materials: tyreMat; castsShadows: car.castsShadows
                eulerRotation.z: 90
                scale: car.sc(car.wheelRadius * 2, 0.205, car.wheelRadius * 2)
            }
            Model {   // rim face
                source: "#Cylinder"; materials: rimMat
                eulerRotation.z: 90
                position: Qt.vector3d(wheel.side * 0.100, 0, 0)
                scale: car.sc(car.wheelRadius * 1.30, 0.012, car.wheelRadius * 1.30)
            }
            Model {   // dark gap between spokes and brake disc behind
                visible: car.detailed
                source: "#Cylinder"; materials: discMat
                eulerRotation.z: 90
                position: Qt.vector3d(wheel.side * 0.088, 0, 0)
                scale: car.sc(car.wheelRadius * 1.22, 0.01, car.wheelRadius * 1.22)
            }
            Repeater3D {   // five spokes
                model: car.detailed ? 5 : 0
                delegate: Model {
                    required property int index
                    source: "#Cube"; materials: rimMat
                    position: Qt.vector3d(wheel.side * 0.108, 0, 0)
                    eulerRotation.x: index * 72
                    scale: car.sc(0.012, car.wheelRadius * 1.3, 0.05)
                }
            }
            Model {   // hub cap
                visible: car.detailed
                source: "#Cylinder"; materials: chromeMat
                eulerRotation.z: 90
                position: Qt.vector3d(wheel.side * 0.114, 0, 0)
                scale: car.sc(0.09, 0.012, 0.09)
            }
        }
    }
    Wheel { position: Qt.vector3d(-car.trackHalf, car.wheelRadius, -car.axleZ); steered: true; side: -1 }
    Wheel { position: Qt.vector3d(car.trackHalf, car.wheelRadius, -car.axleZ); steered: true; side: 1 }
    Wheel { position: Qt.vector3d(-car.trackHalf, car.wheelRadius, car.axleZ); side: -1 }
    Wheel { position: Qt.vector3d(car.trackHalf, car.wheelRadius, car.axleZ); side: 1 }
}
