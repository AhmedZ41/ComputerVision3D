#include "BoundingBox.h"
#include <limits>

// ==============================
// Computes Axis-Aligned Bounding Box (AABB)
// ==============================
// Given a list of 4D points (QVector4D), this function returns a pair:
// - First element: min corner (QVector3D) of the bounding box
// - Second element: max corner (QVector3D) of the bounding box
std::pair<QVector3D, QVector3D> computeBoundingBox(const std::vector<QVector4D>& points) {
    // Initialize minPoint with very large values (will shrink)
    QVector3D minPoint(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());

    // Initialize maxPoint with very small values (will grow)
    QVector3D maxPoint(
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest());

    // Iterate through all points in the input
    for (const auto& p : points) {
        QVector3D v(p.x(), p.y(), p.z());  // Extract the 3D component, ignore w

        // Update minimum coordinates
        minPoint.setX(std::min(minPoint.x(), v.x()));
        minPoint.setY(std::min(minPoint.y(), v.y()));
        minPoint.setZ(std::min(minPoint.z(), v.z()));

        // Update maximum coordinates
        maxPoint.setX(std::max(maxPoint.x(), v.x()));
        maxPoint.setY(std::max(maxPoint.y(), v.y()));
        maxPoint.setZ(std::max(maxPoint.z(), v.z()));
    }

    // Return the pair of min and max corners defining the AABB
    return {minPoint, maxPoint};
}
