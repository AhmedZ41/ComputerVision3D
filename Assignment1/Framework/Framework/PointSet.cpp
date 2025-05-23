#include "PointSet.h"
#include "BoundingBox.h"

// ========================
// Constructor Definitions
// ========================

// Default constructor: creates an empty point set
PointSet::PointSet() {}

// Constructor: initializes the point set with a given vector of 4D points
// and immediately computes its bounding box (AABB)
PointSet::PointSet(const std::vector<QVector4D>& pts)
    : points(pts)
{
    computeBoundingBox();
}

// ========================
// Bounding Box Calculation
// ========================

// Computes the AABB (Axis-Aligned Bounding Box) for the current point set
void PointSet::computeBoundingBox()
{
    // If there are no points, do nothing
    if (points.empty()) return;

    // Use helper function from BoundingBox.h to compute min and max corners
    auto [minC, maxC] = ::computeBoundingBox(points);
    minCorner = minC;
    maxCorner = maxC;
}

// ========================
// Getters
// ========================

// Returns a const reference to the internal point vector
const std::vector<QVector4D>& PointSet::getPoints() const {
    return points;
}

// Returns the computed min corner of the bounding box
const QVector3D& PointSet::getMinCorner() const {
    return minCorner;
}

// Returns the computed max corner of the bounding box
const QVector3D& PointSet::getMaxCorner() const {
    return maxCorner;
}

// Returns the number of points in the set
size_t PointSet::size() const {
    return points.size();
}

// Returns whether the point set is empty
bool PointSet::empty() const {
    return points.empty();
}

// ========================
// Subset Extraction
// ========================

// Extracts a new PointSet containing only points inside a given bounding box
PointSet PointSet::extractSubsetInside(const QVector3D& minC, const QVector3D& maxC) const
{
    std::vector<QVector4D> subset;

    // Loop through all points in the current set
    for (const auto& p : points) {
        QVector3D v(p.x(), p.y(), p.z());  // Extract 3D coordinates

        // Check if the point is inside the specified min-max bounding box
        bool inside = (v.x() >= minC.x() && v.x() <= maxC.x()) &&
                      (v.y() >= minC.y() && v.y() <= maxC.y()) &&
                      (v.z() >= minC.z() && v.z() <= maxC.z());

        if (inside)
            subset.push_back(p);
    }

    // Return a new PointSet with the extracted subset
    return PointSet(subset);
}

// ========================
// Transformations
// ========================

// Applies an affine transformation matrix to all points
void PointSet::affineMap(const QMatrix4x4& matrix) {
    for (auto& point : points) {
        point = matrix * point;  // Apply the matrix to each 4D point
    }

    // After transforming the points, recompute the bounding box
    computeBoundingBox();
}
