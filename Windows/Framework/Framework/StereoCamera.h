#pragma once

#ifndef STEREOCAMERA_H
#define STEREOCAMERA_H

#include "SceneObject.h"
#include "PerspectiveCamera.h"
#include "PointCloud.h"
#include "Hexahedron.h"

#include <vector>
#include <QVector4D>
#include <QVector3D>
#include <QMatrix4x4>
#include <QColor>

// ────────────────────────────────────────────────────────────────
// StereoCamera – aggregates two PerspectiveCamera objects (left & right),
//                 and provides stereo reconstruction with per-object grouping
// ────────────────────────────────────────────────────────────────
class StereoCamera : public SceneObject {
private:
    PerspectiveCamera                        leftCam;
    PerspectiveCamera                        rightCam;
    // one vector per Hexahedron: indexed by original vertex order. w=1 if reconstructed, 0 else.
    // For Part 3: w > 1 indicates error magnitude (w = 1.0 + error, where error ranges 0-1)
    std::vector<std::vector<QVector4D>>      reconstructedPoints;
    std::vector<const Hexahedron*>           hexes;         // parallel to reconstructedPoints

    // utility: midpoint of shortest segment between two skew rays
    QVector3D triangulate(const QVector3D& o1, const QVector3D& d1,
                          const QVector3D& o2, const QVector3D& d2) const;

    // For Part 3: Function to misalign the right camera slightly
    void misalignRightCamera(float angleInDegrees);

public:
    StereoCamera(const PerspectiveCamera& left,
                 const PerspectiveCamera& right);
    ~StereoCamera() override { /* nothing to free */ }

    void affineMap(const QMatrix4x4& matrix) override;
    void draw(const RenderCamera& renderer,
              const QColor&       color      = QColor(255, 0, 255),
              float               lineWidth  = 2.0f) const override;
    std::vector<QVector4D> getPoints() const override;

    // reconstruct only vertices visible in both views, grouped per object
    void reconstruct(const std::vector<SceneObject*>& scene);
    void drawProjections(const RenderCamera&              renderer,
                         const std::vector<SceneObject*>& scene) const;
};

#endif // STEREOCAMERA_H
