#include "BoundingBox.h"
#include <limits>

// Returns a pair (min, max) representing the AABB of the input points
std::pair<QVector3D, QVector3D> computeBoundingBox(const std::vector<QVector4D>& points) {
    // Initialize min and max with extreme values
    QVector3D minPoint(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());

    QVector3D maxPoint(
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest());

    for (const auto& p : points) {
        QVector3D v(p.x(), p.y(), p.z());  // Ignore the 4th component (homogeneous)

        minPoint.setX(std::min(minPoint.x(), v.x()));
        minPoint.setY(std::min(minPoint.y(), v.y()));
        minPoint.setZ(std::min(minPoint.z(), v.z()));

        maxPoint.setX(std::max(maxPoint.x(), v.x()));
        maxPoint.setY(std::max(maxPoint.y(), v.y()));
        maxPoint.setZ(std::max(maxPoint.z(), v.z()));
    }

    return {minPoint, maxPoint};
}
