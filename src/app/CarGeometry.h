#pragma once

#include <QtQuick3D/QQuick3DGeometry>
#include <QtQml/qqmlregistration.h>

// Procedural car body, lofted from smooth side-profile curves (no external model). Units are metres.
// Car frame: x right, y up, z backwards (the front bumper is at -length/2), matching the scene.
//
//   LowerBody: bumper to bumper below the beltline. Subsets: 0 paint, 1 dark lower trim, 2 headlamps, 3 tail lamps,
//              4 left indicators, 5 right indicators, 6 reversing lamps (the lamps are zones of the body surface, so they sit flush).
//   Cabin:     the greenhouse above it.          Subset 0 = roof paint,   subset 1 = glass.
class CarBodyGeometry : public QQuick3DGeometry {
    Q_OBJECT
    QML_NAMED_ELEMENT(CarBodyGeometry)
    Q_PROPERTY(Part part READ part WRITE setPart NOTIFY changed)
    Q_PROPERTY(Style style READ style WRITE setStyle NOTIFY changed)
    Q_PROPERTY(float length READ length WRITE setLength NOTIFY changed)
    Q_PROPERTY(float width READ width WRITE setWidth NOTIFY changed)

public:
    enum Part { LowerBody, Cabin };
    Q_ENUM(Part)
    enum Style { Sedan, Suv, Van };
    Q_ENUM(Style)

    explicit CarBodyGeometry(QQuick3DObject* parent = nullptr);

    Part part() const { return part_; }
    Style style() const { return style_; }
    float length() const { return length_; }
    float width() const { return width_; }
    void setPart(Part p);
    void setStyle(Style s);
    void setLength(float v);
    void setWidth(float v);

signals:
    void changed();

private:
    void rebuild();

    Part part_ = LowerBody;
    Style style_ = Sedan;
    float length_ = 4.5f;
    float width_ = 1.85f;
};
