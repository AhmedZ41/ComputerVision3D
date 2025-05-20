#include "PointSet.h"
#include "BoundingBox.h"  // we use the function from Subtask 2

PointSet::PointSet() {}

PointSet::PointSet(const std::vector<QVector4D>& pts)
    : points(pts)
{
    computeBoundingBox();
}

void PointSet::computeBoundingBox()
{
    if (points.empty()) return;

    auto [minC, maxC] = ::computeBoundingBox(points);
    minCorner = minC;
    maxCorner = maxC;
}

const std::vector<QVector4D>& PointSet::getPoints() const {
    return points;
}

const QVector3D& PointSet::getMinCorner() const {
    return minCorner;
}

const QVector3D& PointSet::getMaxCorner() const {
    return maxCorner;
}

size_t PointSet::size() const {
    return points.size();
}

bool PointSet::empty() const {
    return points.empty();
}

PointSet PointSet::extractSubsetInside(const QVector3D& minC, const QVector3D& maxC) const
{
    std::vector<QVector4D> subset;

    for (const auto& p : points) {
        QVector3D v(p.x(), p.y(), p.z());

        bool inside = (v.x() >= minC.x() && v.x() <= maxC.x()) &&
                      (v.y() >= minC.y() && v.y() <= maxC.y()) &&
                      (v.z() >= minC.z() && v.z() <= maxC.z());

        if (inside) subset.push_back(p);
    }

    return PointSet(subset);
}
