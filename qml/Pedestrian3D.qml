import QtQuick
import QtQuick3D

// Simple articulated pedestrian (about 1.75 m). Walks with a leg and arm swing driven by distance walked.
Node {
    id: ped
    property bool walking: false
    property real phase: 0            // distance walked, m
    property color jacket: "#c0392b"
    property color trousers: "#2c3e50"
    property real facing: 90          // degrees about Y: 90 = towards -X (the left), -90 = towards +X

    readonly property real swing: walking ? 32 * Math.sin(2 * Math.PI * phase / 1.5) : 0
    function sc(w, h, d) { return Qt.vector3d(w / 100, h / 100, d / 100) }

    PrincipledMaterial { id: skin; baseColor: "#d9a98a"; roughness: 0.7 }
    PrincipledMaterial { id: hair; baseColor: "#2b1d14"; roughness: 0.9 }
    PrincipledMaterial { id: coat; baseColor: ped.jacket; roughness: 0.8 }
    PrincipledMaterial { id: legs; baseColor: ped.trousers; roughness: 0.85 }
    PrincipledMaterial { id: shoe; baseColor: "#1a1a1a"; roughness: 0.7 }
    PrincipledMaterial { id: vest; baseColor: "#ffd21f"; roughness: 0.6; emissiveFactor: Qt.vector3d(0.25, 0.22, 0.0) }

    Node {
        eulerRotation.y: ped.facing
        // body: slight bob while walking
        y: ped.walking ? 0.02 * Math.abs(Math.sin(2 * Math.PI * ped.phase / 1.5)) : 0

        Model { source: "#Cylinder"; materials: coat; castsShadows: true; position: Qt.vector3d(0, 1.22, 0); scale: ped.sc(0.38, 0.62, 0.24) }
        Model { source: "#Cylinder"; materials: vest; position: Qt.vector3d(0, 1.24, 0); scale: ped.sc(0.395, 0.34, 0.255) }   // high-visibility vest
        Model { source: "#Sphere"; materials: coat; castsShadows: true; position: Qt.vector3d(0, 1.50, 0); scale: ped.sc(0.44, 0.16, 0.26) }  // shoulders
        Model { source: "#Cylinder"; materials: skin; position: Qt.vector3d(0, 1.57, 0); scale: ped.sc(0.09, 0.08, 0.09) }                 // neck
        Model { source: "#Sphere"; materials: skin; castsShadows: true; position: Qt.vector3d(0, 1.68, 0); scale: ped.sc(0.19, 0.23, 0.21) }  // head
        Model { source: "#Sphere"; materials: hair; position: Qt.vector3d(0, 1.72, 0.015); scale: ped.sc(0.20, 0.17, 0.22) }                // hair

        // legs and arms hang from pivots at hip and shoulder
        Repeater3D {
            model: [-1, 1]
            delegate: Node {
                id: limb
                required property int modelData
                // hip pivot
                Node {
                    position: Qt.vector3d(limb.modelData * 0.09, 0.90, 0)
                    eulerRotation.z: 0
                    eulerRotation.x: limb.modelData * ped.swing
                    Model { source: "#Cylinder"; materials: legs; castsShadows: true; position: Qt.vector3d(0, -0.42, 0); scale: ped.sc(0.14, 0.86, 0.15) }
                    Model { source: "#Cube"; materials: shoe; position: Qt.vector3d(0, -0.87, -0.04); scale: ped.sc(0.13, 0.07, 0.27) }
                }
                // shoulder pivot (arms swing against the legs)
                Node {
                    position: Qt.vector3d(limb.modelData * 0.235, 1.46, 0)
                    eulerRotation.x: -limb.modelData * ped.swing * 0.8
                    Model { source: "#Cylinder"; materials: coat; castsShadows: true; position: Qt.vector3d(0, -0.26, 0); scale: ped.sc(0.09, 0.52, 0.09) }
                    Model { source: "#Sphere"; materials: skin; position: Qt.vector3d(0, -0.55, 0); scale: ped.sc(0.085, 0.085, 0.085) }
                }
            }
        }
    }
}
