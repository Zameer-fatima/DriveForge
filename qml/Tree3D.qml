import QtQuick
import QtQuick3D

// Roadside tree built from primitives. species 0 = pine (stacked cones), 1 = broadleaf (clustered crowns).
Node {
    id: tree
    property int species: 0
    property real size: 1.0           // overall scale
    property real tint: 0.0           // -1 .. 1 varies the greens a little

    function sc(w, h, d) { return Qt.vector3d(w / 100, h / 100, d / 100) }
    function green(r, g, b) { return Qt.rgba(Math.max(0, r + 0.06 * tint), Math.max(0, g + 0.10 * tint), Math.max(0, b), 1) }

    PrincipledMaterial { id: bark; baseColor: "#4a3524"; roughness: 1 }
    PrincipledMaterial { id: leafA; baseColor: tree.green(0.12, 0.30, 0.13); roughness: 1 }
    PrincipledMaterial { id: leafB; baseColor: tree.green(0.18, 0.38, 0.14); roughness: 1 }
    PrincipledMaterial { id: leafC; baseColor: tree.green(0.24, 0.45, 0.16); roughness: 1 }

    scale: Qt.vector3d(size, size, size)

    Model { source: "#Cylinder"; materials: bark; castsShadows: true; position: Qt.vector3d(0, 1.3, 0); scale: tree.sc(species === 0 ? 0.34 : 0.5, 2.6, species === 0 ? 0.34 : 0.5) }

    // pine
    Node {
        visible: tree.species === 0
        Model { source: "#Cone"; materials: leafA; castsShadows: true; position: Qt.vector3d(0, 2.6, 0); scale: tree.sc(3.0, 3.0, 3.0) }
        Model { source: "#Cone"; materials: leafB; castsShadows: true; position: Qt.vector3d(0, 4.1, 0); scale: tree.sc(2.4, 2.8, 2.4) }
        Model { source: "#Cone"; materials: leafC; castsShadows: true; position: Qt.vector3d(0, 5.5, 0); scale: tree.sc(1.7, 2.6, 1.7) }
    }
    // broadleaf
    Node {
        visible: tree.species === 1
        Model { source: "#Sphere"; materials: leafA; castsShadows: true; position: Qt.vector3d(0, 3.6, 0); scale: tree.sc(4.4, 3.8, 4.4) }
        Model { source: "#Sphere"; materials: leafB; castsShadows: true; position: Qt.vector3d(0.9, 4.4, 0.5); scale: tree.sc(3.2, 2.8, 3.2) }
        Model { source: "#Sphere"; materials: leafC; castsShadows: true; position: Qt.vector3d(-0.8, 4.6, -0.4); scale: tree.sc(3.0, 2.6, 3.0) }
        Model { source: "#Sphere"; materials: leafB; castsShadows: true; position: Qt.vector3d(0.1, 5.4, 0.1); scale: tree.sc(2.4, 2.0, 2.4) }
    }
}
