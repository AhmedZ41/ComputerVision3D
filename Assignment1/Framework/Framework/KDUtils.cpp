#include "KDUtils.h"
#include <algorithm>

// Sorts the points in-place by a specific axis
void sortPointsByAxis(std::vector<QVector4D>& points, int axis) {
    std::sort(points.begin(), points.end(), [axis](const QVector4D& a, const QVector4D& b) {
        return a[axis] < b[axis];
    });
}

// Returns the coordinate value of the median point along the given axis
float getMedianSplitValue(std::vector<QVector4D>& points, int axis) {
    if (points.empty()) return 0.0f;
    sortPointsByAxis(points, axis);
    return points[points.size() / 2][axis];
}
