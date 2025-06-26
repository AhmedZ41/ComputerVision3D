#pragma once

#include "SceneObject.h"
#include <Eigen/Dense>
#include <QVector3D>
#include <array>

class PCAAxes : public SceneObject {
public:
    PCAAxes(const QVector4D& centroid, const Eigen::Matrix3f& eigenvectors, float scale = 0.3f);

    void draw(const RenderCamera& camera, const QColor& color, float lineWidth) const override;
    void affineMap(const QMatrix4x4& m) override;

private:
    QVector3D origin;
    std::array<QVector3D, 3> directions;
    float scale;
};
