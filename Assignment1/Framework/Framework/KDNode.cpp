#include "KDNode.h"
#include <algorithm>

KDNode::KDNode(const PointSet& pointSet, int depth, int maxDepth, int minPoints)
    : data(pointSet)
{
    // Stop if we reached max depth or too few points
    if (depth >= maxDepth || data.size() <= minPoints) return;

    // Determine axis: cycle through x, y, z (0,1,2)
    axis = depth % 3;

    // Sort points based on axis to find median
    std::vector<QVector4D> sortedPoints = data.getPoints();
    std::sort(sortedPoints.begin(), sortedPoints.end(), [this](const QVector4D& a, const QVector4D& b) {
        return a[axis] < b[axis];
    });

    int medianIndex = sortedPoints.size() / 2;
    splitValue = sortedPoints[medianIndex][axis];

    // Create left and right subsets
    std::vector<QVector4D> leftPoints, rightPoints;
    for (const auto& p : sortedPoints) {
        if (p[axis] <= splitValue)
            leftPoints.push_back(p);
        else
            rightPoints.push_back(p);
    }

    // Create children
    if (!leftPoints.empty())  left  = new KDNode(PointSet(leftPoints),  depth + 1, maxDepth, minPoints);
    if (!rightPoints.empty()) right = new KDNode(PointSet(rightPoints), depth + 1, maxDepth, minPoints);
}

KDNode::~KDNode() {
    delete left;
    delete right;
}
