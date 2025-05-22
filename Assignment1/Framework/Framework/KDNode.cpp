#include "RenderCamera.h"  // for renderer.renderPlane
#include "KDNode.h"
#include <algorithm>
#include <QColor>


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


void KDNode::draw(const RenderCamera& renderer, const QColor& color, float lineWidth, int currentDepth) const {
    if (currentDepth >= 3) return; // only draw first 3 levels

    auto minC = data.getMinCorner();
    auto maxC = data.getMaxCorner();

    // Calculate 4 corners of the splitting plane
    QVector3D a, b, c, d;

    if (axis == 0) { // Split along X
        a = QVector3D(splitValue, minC.y(), minC.z());
        b = QVector3D(splitValue, maxC.y(), minC.z());
        c = QVector3D(splitValue, maxC.y(), maxC.z());
        d = QVector3D(splitValue, minC.y(), maxC.z());
    } else if (axis == 1) { // Split along Y
        a = QVector3D(minC.x(), splitValue, minC.z());
        b = QVector3D(maxC.x(), splitValue, minC.z());
        c = QVector3D(maxC.x(), splitValue, maxC.z());
        d = QVector3D(minC.x(), splitValue, maxC.z());
    } else if (axis == 2) { // Split along Z
        a = QVector3D(minC.x(), minC.y(), splitValue);
        b = QVector3D(maxC.x(), minC.y(), splitValue);
        c = QVector3D(maxC.x(), maxC.y(), splitValue);
        d = QVector3D(minC.x(), maxC.y(), splitValue);
    }

    renderer.renderPlane(a, b, c, d, color, 0.3f); // draw plane with some transparency

    // Recursively draw children
    if (left)  left->draw(renderer, QColor(0, 255, 0), lineWidth, currentDepth + 1);  // green
    if (right) right->draw(renderer, QColor(255, 0, 0), lineWidth, currentDepth + 1); // red
}

void KDNode::affineMap(const QMatrix4x4& matrix) {
    data.affineMap(matrix);
    if (left)  left->affineMap(matrix);
    if (right) right->affineMap(matrix);
}

// This is the base-class-required draw method
void KDNode::draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const {
    draw(renderer, color, lineWidth, 0);  // Call recursive one with default depth = 0
}


