#include "PCAAxes.h"

PCAAxes::PCAAxes(const QVector4D& centroid, const Eigen::Matrix3f& eigenvectors, float scale)
    : origin(centroid.toVector3D()), scale(scale)
{
    for (int i = 0; i < 3; ++i) {
        directions[i] = QVector3D(eigenvectors(0, i), eigenvectors(1, i), eigenvectors(2, i));
    }
    type = SceneObjectType::ST_PCA_AXES;
}

void PCAAxes::draw(const RenderCamera& camera, const QColor&, float) const {
    camera.renderLine(origin, origin + scale * directions[2], Qt::red);
    camera.renderLine(origin, origin + scale * directions[1], Qt::green);
    camera.renderLine(origin, origin + scale * directions[0], Qt::blue);
}

void PCAAxes::affineMap(const QMatrix4x4& m) {
    origin = m * origin;
    for (auto& dir : directions)
        dir = m.mapVector(dir);
}
