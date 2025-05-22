#include "OctreeNode.h"

OctreeNode::OctreeNode(const QVector<QVector4D>& points,
                       const QVector4D& minCorner,
                       const QVector4D& maxCorner,
                       int depth)
{
    type = SceneObjectType::ST_CUBE;
    bboxMin = minCorner;
    bboxMax = maxCorner;
    children.fill(nullptr);

    if (depth >= 3 || points.size() < 20) return;

    QVector<QVector4D> childPoints[8];
    QVector4D mid = 0.5f * (bboxMin + bboxMax);

    for (const auto& p : points) {
        int index = (p.x() > mid.x()) + 2 * (p.y() > mid.y()) + 4 * (p.z() > mid.z());
        childPoints[index].push_back(p);
    }

    for (int i = 0; i < 8; ++i) {
        QVector4D childMin = bboxMin;
        QVector4D childMax = mid;
        if (i & 1) { childMin.setX(mid.x()); childMax.setX(bboxMax.x()); }
        if (i & 2) { childMin.setY(mid.y()); childMax.setY(bboxMax.y()); }
        if (i & 4) { childMin.setZ(mid.z()); childMax.setZ(bboxMax.z()); }

        if (!childPoints[i].isEmpty())
            children[i] = new OctreeNode(childPoints[i], childMin, childMax, depth + 1);
    }
}

OctreeNode::~OctreeNode() {
    for (auto* child : children)
        delete child;
}

void OctreeNode::draw(const RenderCamera& camera, const QColor& baseColor, float lineWidth) const {
    // Determine current depth
    static int currentDepth = 0;

    // Assign color based on depth: X → red, Y → green, Z → blue
    QColor color;
    switch (currentDepth % 3) {
    case 0: color = QColor(255, 0, 0); break;   // X axis split: red
    case 1: color = QColor(0, 255, 0); break;   // Y axis split: green
    case 2: color = QColor(0, 0, 255); break;   // Z axis split: blue
    }

    camera.renderWireCube(bboxMin.toVector3D(), bboxMax.toVector3D(), color, lineWidth);

    currentDepth++;
    for (const auto* child : children) {
        if (child) child->draw(camera, color, lineWidth);
    }
    currentDepth--;
}


void OctreeNode::affineMap(const QMatrix4x4&) {
}
