#include "OctreeNode.h"
#include "RenderCamera.h"
#include <QColor>

OctreeNode::OctreeNode(const QVector<QVector4D>& pts, const QVector4D& minCorner, const QVector4D& maxCorner, int depth) {
    points = pts;
    bboxMin = minCorner;
    bboxMax = maxCorner;

    if (depth >= 3 || points.size() < 20) return;

    QVector<QVector4D> childPoints[8];

    QVector4D mid = 0.5f * (bboxMin + bboxMax);

    for (const auto& p : points) {
        int index = (p.x() > mid.x()) + 2 * (p.y() > mid.y()) + 4 * (p.z() > mid.z());
        childPoints[index].push_back(p);
    }

    for (int i = 0; i < 8; ++i) {
        QVector4D minC = bboxMin;
        QVector4D maxC = mid;

        if (i & 1) minC.setX(mid.x()), maxC.setX(bboxMax.x());
        if (i & 2) minC.setY(mid.y()), maxC.setY(bboxMax.y());
        if (i & 4) minC.setZ(mid.z()), maxC.setZ(bboxMax.z());

        if (!childPoints[i].isEmpty()) {
            children[i] = new OctreeNode(childPoints[i], minC, maxC, depth + 1);
        }
    }
}

OctreeNode::~OctreeNode() {
    for (auto& c : children)
        delete c;
}

void OctreeNode::draw(const RenderCamera& camera, const QColor& color, float lineWidth) const {
    camera.renderBox(QVector3D(bboxMin.toVector3D()), QVector3D(bboxMax.toVector3D()), color, lineWidth);

    for (auto& c : children)
        if (c) c->draw(camera, color, lineWidth);
}

void OctreeNode::affineMap(const QMatrix4x4&) {
    // Not needed for now
}
