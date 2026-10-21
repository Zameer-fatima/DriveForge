#include "CarGeometry.h"

#include <QVector3D>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Monotone cubic (Fritsch-Carlson) through control points: smooth, never overshoots.
struct Curve {
    std::vector<double> x, y, m;

    Curve(std::initializer_list<std::pair<double, double>> pts) {
        for (const auto& p : pts) { x.push_back(p.first); y.push_back(p.second); }
        const std::size_t n = x.size();
        std::vector<double> d(n - 1);
        for (std::size_t i = 0; i + 1 < n; ++i) d[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
        m.assign(n, 0.0);
        m[0] = d[0];
        m[n - 1] = d[n - 2];
        for (std::size_t i = 1; i + 1 < n; ++i) m[i] = d[i - 1] * d[i] <= 0.0 ? 0.0 : 2.0 * d[i - 1] * d[i] / (d[i - 1] + d[i]);
        for (std::size_t i = 0; i + 1 < n; ++i) {
            if (d[i] == 0.0) { m[i] = m[i + 1] = 0.0; continue; }
            const double a = m[i] / d[i], b = m[i + 1] / d[i];
            const double h = std::hypot(a, b);
            if (h > 3.0) { m[i] = 3.0 * a / h * d[i]; m[i + 1] = 3.0 * b / h * d[i]; }
        }
    }

    double at(double s) const {
        const std::size_t n = x.size();
        if (s <= x.front()) return y.front();
        if (s >= x.back()) return y.back();
        std::size_t i = 0;
        while (i + 2 < n && s > x[i + 1]) ++i;
        const double h = x[i + 1] - x[i], t = (s - x[i]) / h;
        const double t2 = t * t, t3 = t2 * t;
        return (2 * t3 - 3 * t2 + 1) * y[i] + (t3 - 2 * t2 + t) * h * m[i] + (-2 * t3 + 3 * t2) * y[i + 1] + (t3 - t2) * h * m[i + 1];
    }
};

struct StyleDef {
    Curve topY, botY, halfW;   // lower body: side profile and half-width as a fraction of width / 2 (s = 0 front .. 1 rear)
    double exponent;           // cross-section squareness (superellipse)
    double cabStart, cabEnd;   // greenhouse extent along s
    double cabBase;            // height where the greenhouse starts
    Curve cabTop;              // roof line
    double cabTumble;          // sideways taper of the greenhouse
    double roofFrom, roofTo;   // painted roof extent
    double trimHeight;         // height of the dark lower trim above the body bottom
    std::vector<std::pair<double, double>> pillars;  // painted bands (s ranges) on the sides of the greenhouse
};

const StyleDef& styleDef(CarBodyGeometry::Style s) {
    static const StyleDef sedan{
        {{0, 0.74}, {0.02, 0.77}, {0.06, 0.80}, {0.16, 0.84}, {0.28, 0.89}, {0.36, 0.93}, {0.50, 0.95}, {0.70, 0.96}, {0.84, 0.98}, {0.94, 0.95}, {0.985, 0.90}, {1, 0.84}},
        {{0, 0.36}, {0.02, 0.31}, {0.06, 0.29}, {0.94, 0.29}, {0.98, 0.31}, {1, 0.36}},
        {{0, 0.82}, {0.02, 0.90}, {0.08, 0.97}, {0.2, 1.0}, {0.5, 1.0}, {0.8, 1.0}, {0.94, 0.97}, {0.99, 0.90}, {1, 0.84}},
        5.5, 0.34, 0.915, 0.93,
        {{0.34, 0.93}, {0.38, 1.02}, {0.44, 1.17}, {0.51, 1.31}, {0.57, 1.40}, {0.64, 1.445}, {0.72, 1.45}, {0.79, 1.41}, {0.85, 1.29}, {0.89, 1.12}, {0.915, 0.97}},
        0.22, 0.50, 0.82, 0.08,
        {{0.34, 0.385}, {0.585, 0.625}, {0.80, 0.90}}};
    static const StyleDef suv{
        {{0, 0.90}, {0.02, 0.93}, {0.06, 0.97}, {0.20, 1.02}, {0.32, 1.06}, {0.42, 1.09}, {0.60, 1.10}, {0.90, 1.10}, {0.98, 1.06}, {1, 1.00}},
        {{0, 0.46}, {0.02, 0.40}, {0.06, 0.36}, {0.94, 0.36}, {0.98, 0.40}, {1, 0.46}},
        {{0, 0.84}, {0.02, 0.92}, {0.08, 0.98}, {0.22, 1.0}, {0.5, 1.0}, {0.82, 1.0}, {0.94, 0.98}, {0.99, 0.92}, {1, 0.86}},
        5.0, 0.30, 0.985, 1.05,
        {{0.30, 1.05}, {0.36, 1.20}, {0.44, 1.45}, {0.52, 1.62}, {0.60, 1.70}, {0.80, 1.72}, {0.92, 1.68}, {0.97, 1.50}, {0.985, 1.15}},
        0.18, 0.50, 0.90, 0.10,
        {{0.30, 0.34}, {0.545, 0.58}, {0.80, 0.84}, {0.93, 0.985}}};
    static const StyleDef van{
        {{0, 0.96}, {0.02, 1.0}, {0.08, 1.05}, {0.15, 1.07}, {0.30, 1.08}, {0.60, 1.10}, {0.95, 1.10}, {1, 1.04}},
        {{0, 0.50}, {0.02, 0.44}, {0.07, 0.40}, {0.93, 0.40}, {0.97, 0.43}, {1, 0.48}},
        {{0, 0.88}, {0.02, 0.94}, {0.10, 0.98}, {0.22, 1.0}, {0.5, 1.0}, {0.9, 1.0}, {1, 0.94}},
        5.0, 0.16, 0.995, 1.05,
        {{0.16, 1.05}, {0.20, 1.35}, {0.27, 1.78}, {0.34, 1.96}, {0.45, 2.04}, {0.90, 2.06}, {0.97, 2.0}, {0.995, 1.5}},
        0.08, 0.34, 0.97, 0.10,
        {{0.16, 0.20}, {0.55, 0.58}, {0.90, 0.995}}};
    return s == CarBodyGeometry::Suv ? suv : s == CarBodyGeometry::Van ? van : sedan;
}

struct Vtx { QVector3D p, n; float u, v; };

}  // namespace

CarBodyGeometry::CarBodyGeometry(QQuick3DObject* parent) : QQuick3DGeometry(parent) { rebuild(); }

void CarBodyGeometry::setPart(Part p) { if (p != part_) { part_ = p; rebuild(); emit changed(); } }
void CarBodyGeometry::setStyle(Style s) { if (s != style_) { style_ = s; rebuild(); emit changed(); } }
void CarBodyGeometry::setLength(float v) { if (v > 0.5f && v != length_) { length_ = v; rebuild(); emit changed(); } }
void CarBodyGeometry::setWidth(float v) { if (v > 0.5f && v != width_) { width_ = v; rebuild(); emit changed(); } }

void CarBodyGeometry::rebuild() {
    const StyleDef& st = styleDef(style_);
    const bool cabin = part_ == Cabin;
    constexpr int kStations = 120, kRing = 64;
    const double L = length_, halfWidth = width_ / 2.0;

    // ---- grid of positions: [station][ring] ----
    std::vector<QVector3D> grid(static_cast<std::size_t>(kStations) * kRing);
    std::vector<double> stationS(kStations);
    auto cell = [&](int i, int j) -> QVector3D& { return grid[static_cast<std::size_t>(i) * kRing + static_cast<std::size_t>(j)]; };

    for (int i = 0; i < kStations; ++i) {
        const double t = static_cast<double>(i) / (kStations - 1);
        // Cluster stations towards the ends where the surface curves fastest.
        const double s = cabin ? st.cabStart + (st.cabEnd - st.cabStart) * t : 0.5 - 0.5 * std::cos(kPi * t);
        stationS[i] = s;
        const double z = -L / 2.0 + s * L;

        // Rounded tips: the section shrinks like an ellipse over the first and last few centimetres.
        const double tip = 0.03 * 4.5 / L;
        const double edge = cabin ? std::min(t, 1.0 - t) / 0.02 : std::min(s, 1.0 - s) / tip;
        const double round = edge >= 1.0 ? 1.0 : std::sqrt(std::max(0.0, 1.0 - (1.0 - edge) * (1.0 - edge)));
        const double hwBody = st.halfW.at(s) * halfWidth;

        if (!cabin) {
            const double yt = st.topY.at(s), yb = st.botY.at(s);
            const double yc = 0.5 * (yt + yb), hh = 0.5 * (yt - yb) * std::max(round, 0.04);
            const double hw = hwBody * std::max(round, 0.04);
            for (int j = 0; j < kRing; ++j) {
                const double a = 2.0 * kPi * j / kRing;
                const double c = std::cos(a), sn = std::sin(a);
                const double e = 2.0 / st.exponent;
                double x = hw * (c < 0 ? -1 : 1) * std::pow(std::abs(c), e);
                const double y = yc + hh * (sn < 0 ? -1 : 1) * std::pow(std::abs(sn), e);
                if (sn > 0) x *= 1.0 - 0.07 * sn * sn;  // slight tumblehome above the waist
                cell(i, j) = QVector3D(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
            }
        } else {
            const double base = st.cabBase;
            const double top = std::max(st.cabTop.at(s), base);
            const double h = top - base;
            // Greenhouse is narrower than the body and tapers towards its ends.
            const double u = (s - st.cabStart) / (st.cabEnd - st.cabStart);
            const double taper = 0.78 + 0.16 * std::sin(kPi * std::min(std::max(u, 0.0), 1.0));
            const double hw = std::min(hwBody * 0.93, hwBody * taper) * std::max(round, 0.02);
            for (int j = 0; j < kRing; ++j) {
                // Open ring: from the right side (below the base, hidden inside the body), over the roof, to the left side.
                const double a = kPi * j / (kRing - 1);
                const double c = std::cos(a), sn = std::sin(a);
                const double e = 2.0 / 3.4;
                const double x = hw * (c < 0 ? -1 : 1) * std::pow(std::abs(c), e) * (1.0 - st.cabTumble * 0.5 * sn * sn);
                const double y = base + h * std::pow(std::abs(sn), e);
                cell(i, j) = QVector3D(static_cast<float>(x), static_cast<float>(y - (j == 0 || j == kRing - 1 ? 0.05 : 0.0)), static_cast<float>(z));
            }
        }
    }

    // ---- triangles ----
    const int ringQuads = cabin ? kRing - 1 : kRing;
    // Subsets. Lower body: 0 paint, 1 trim, 2 headlamp, 3 tail lamp, 4 indicator left, 5 indicator right, 6 reversing lamp.
    //                Cabin: 0 paint (roof, pillars, sills), 1 glass.
    const int subsetCount = cabin ? 2 : 7;
    std::vector<std::vector<quint32>> idx(static_cast<std::size_t>(subsetCount));
    std::vector<QVector3D> normals(grid.size(), QVector3D(0, 0, 0));

    auto tri = [&](quint32 a, quint32 b, quint32 c, std::vector<quint32>& out) {
        const QVector3D n = QVector3D::crossProduct(grid[b] - grid[a], grid[c] - grid[a]);
        normals[a] += n; normals[b] += n; normals[c] += n;
        out.push_back(a); out.push_back(b); out.push_back(c);
    };
    for (int i = 0; i + 1 < kStations; ++i) {
        for (int j = 0; j < ringQuads; ++j) {
            const int j2 = cabin ? j + 1 : (j + 1) % kRing;
            const quint32 a = static_cast<quint32>(i * kRing + j), b = static_cast<quint32>(i * kRing + j2);
            const quint32 c = static_cast<quint32>((i + 1) * kRing + j), d = static_cast<quint32>((i + 1) * kRing + j2);
            const QVector3D centre = (grid[a] + grid[b] + grid[c] + grid[d]) * 0.25f;
            const QVector3D nrm = QVector3D::crossProduct(grid[b] - grid[a], grid[c] - grid[a]).normalized();  // outward
            const double s = 0.5 * (stationS[i] + stationS[i + 1]);
            int subset = 0;
            // Zones are defined on the mesh grid (station = position along the car, ring angle around it), so their edges follow
            // grid lines and stay smooth. alpha is the elevation of the quad around the section, measured from the side line:
            // 0 = side wall, +90 = top, -90 = bottom, the same on both sides.
            const double ring = 360.0 * (j + 0.5) / kRing;
            const bool rightSide = !cabin ? (ring < 90.0 || ring > 270.0) : true;
            double alpha;
            if (!cabin) {
                alpha = rightSide ? (ring > 270.0 ? ring - 360.0 : ring) : 180.0 - ring;
            } else {
                const double a = 180.0 * (j + 0.5) / (kRing - 1);  // 0 = right base .. 90 = roof centre .. 180 = left base
                alpha = a <= 90.0 ? 90.0 - a : a - 90.0;           // distance from the roof centre line
            }
            if (!cabin) {
                const double zf = s * L, zr = (1.0 - s) * L;  // distance from the front / rear tip
                // Sideways position as a fraction of the full half-width: keeps lamps at the corners, never a wedge across the face.
                const double ax = std::abs(static_cast<double>(centre.x())) / (st.halfW.at(s) * halfWidth);
                if (centre.y() < st.botY.at(s) + st.trimHeight) {
                    subset = 1;  // dark lower trim
                } else if (zf < 0.26 && alpha > 16.0 && alpha < 66.0 && ax > 0.38 && ax < 0.97) {
                    subset = ax > 0.80 ? (rightSide ? 5 : 4) : 2;     // headlamp; indicator at the very corner
                } else if (zr < 0.26 && alpha > 16.0 && alpha < 66.0 && ax > 0.38 && ax < 0.97) {
                    subset = ax > 0.82 ? (rightSide ? 5 : 4) : 3;     // tail lamp; indicator at the very corner
                } else if (zr < 0.14 && alpha > -10.0 && alpha < 2.0 && ax > 0.50 && ax < 0.72) {
                    subset = 6;                                       // reversing lamp low in the bumper
                }
            } else {
                // alpha here: small = near the roof centre line, large = down the side.
                const bool roofZone = alpha < 33.0 && s > st.roofFrom && s < st.roofTo;
                bool pillar = false;
                for (const auto& band : st.pillars) pillar = pillar || (s >= band.first && s <= band.second);
                const bool sideZone = alpha > 40.0;                   // the vertical window area (not windscreen / roof)
                const bool sill = (j == 0 || j == kRing - 2) || alpha > 80.0;
                subset = (roofZone || (pillar && sideZone) || sill) ? 0 : 1;
            }
            std::vector<quint32>& out = idx[static_cast<std::size_t>(subset)];
            tri(a, b, c, out);  // counter-clockwise seen from outside: front faces point out, and so do the normals
            tri(b, d, c, out);
        }
    }
    for (auto& v : idx)
        if (v.empty()) { v.push_back(0); v.push_back(0); v.push_back(0); }  // keep every subset valid (one degenerate triangle)

    // ---- pack vertices ----
    std::vector<Vtx> verts(grid.size());
    for (int i = 0; i < kStations; ++i) {
        for (int j = 0; j < kRing; ++j) {
            const std::size_t k = static_cast<std::size_t>(i) * kRing + static_cast<std::size_t>(j);
            QVector3D n = normals[k];
            n = n.lengthSquared() > 1e-12f ? n.normalized() : QVector3D(0, 1, 0);
            verts[k] = {grid[k], n, static_cast<float>(i) / (kStations - 1), static_cast<float>(j) / (kRing - 1)};
        }
    }
    QByteArray vdata(static_cast<qsizetype>(verts.size() * sizeof(Vtx)), Qt::Uninitialized);
    std::memcpy(vdata.data(), verts.data(), static_cast<std::size_t>(vdata.size()));
    std::vector<quint32> all;
    for (const auto& v : idx) all.insert(all.end(), v.begin(), v.end());
    QByteArray idata(static_cast<qsizetype>(all.size() * sizeof(quint32)), Qt::Uninitialized);
    std::memcpy(idata.data(), all.data(), static_cast<std::size_t>(idata.size()));

    QVector3D lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
    for (const QVector3D& p : grid) {
        lo = QVector3D(std::min(lo.x(), p.x()), std::min(lo.y(), p.y()), std::min(lo.z(), p.z()));
        hi = QVector3D(std::max(hi.x(), p.x()), std::max(hi.y(), p.y()), std::max(hi.z(), p.z()));
    }

    clear();
    setStride(sizeof(Vtx));
    setPrimitiveType(PrimitiveType::Triangles);
    addAttribute(Attribute::PositionSemantic, 0, Attribute::F32Type);
    addAttribute(Attribute::NormalSemantic, sizeof(QVector3D), Attribute::F32Type);
    addAttribute(Attribute::TexCoord0Semantic, 2 * sizeof(QVector3D), Attribute::F32Type);
    addAttribute(Attribute::IndexSemantic, 0, Attribute::U32Type);
    setVertexData(vdata);
    setIndexData(idata);
    int offset = 0;
    for (std::size_t k = 0; k < idx.size(); ++k) {
        addSubset(offset, static_cast<int>(idx[k].size()), lo, hi, QStringLiteral("part%1").arg(k));
        offset += static_cast<int>(idx[k].size());
    }
    setBounds(lo, hi);
    update();
}
