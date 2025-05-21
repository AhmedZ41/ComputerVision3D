#include "OctreeNode.h"
#include "RenderCamera.h"
#include <QColor>

OctreeNode::OctreeNode(const PointSet& pointSet, int d, int maxDepth)
    : data(pointSet), depth(d)
{
    type = SceneObjectType::ST_OCTREE;
    minCorner = data.getMinCorner();
    maxCorner = data.getMaxCorner();
    children.fill(nullptr);

    if (depth < maxDepth && data.size() > 1) {
        buildChildren(maxDepth);
    }
}

OctreeNode::~OctreeNode() {
    for (auto child : children)
        delete child;
}

void OctreeNode::buildChildren(int maxDepth) {
    QVector3D center = (minCorner + maxCorner) * 0.5f;

    std::vector<QVector4D> allPoints = data.getPoints();
    std::array<std::vector<QVector4D>, 8> octantPoints;

    // Assign points to 1 of 8 octants
    for (const auto& p : allPoints) {
        int index = 0;
        if (p.x() > center.x()) index |= 1;
        if (p.y() > center.y()) index |= 2;
        if (p.z() > center.z()) index |= 4;
        octantPoints[index].push_back(p);
    }

    // Recursively build children
    for (int i = 0; i < 8; ++i) {
        if (!octantPoints[i].empty()) {
            children[i] = new OctreeNode(PointSet(octantPoints[i]), depth + 1, maxDepth);
        }
    }
}

void OctreeNode::draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const {
    if (depth >= 3) return;  // Only visualize first 3 levels

    // draw bounding box
    QVector3D a(minCorner.x(), minCorner.y(), minCorner.z());
    QVector3D b(maxCorner.x(), minCorner.y(), minCorner.z());
    QVector3D c(maxCorner.x(), maxCorner.y(), minCorner.z());
    QVector3D d(minCorner.x(), maxCorner.y(), minCorner.z());
    QVector3D e(minCorner.x(), minCorner.y(), maxCorner.z());
    QVector3D f(maxCorner.x(), minCorner.y(), maxCorner.z());
    QVector3D g(maxCorner.x(), maxCorner.y(), maxCorner.z());
    QVector3D h(minCorner.x(), maxCorner.y(), maxCorner.z());

    std::vector<std::pair<QVector3D, QVector3D>> edges = {
        {a,b},{b,c},{c,d},{d,a},
        {e,f},{f,g},{g,h},{h,e},
        {a,e},{b,f},{c,g},{d,h}
    };

    for (const auto& edge : edges)
        renderer.renderLine(edge.first, edge.second, color, lineWidth);

    // draw children
    for (auto child : children)
        if (child) child->draw(renderer, color, lineWidth);
}

void OctreeNode::affineMap(const QMatrix4x4& matrix) {
    data.affineMap(matrix);
    minCorner = matrix * minCorner;
    maxCorner = matrix * maxCorner;
    for (auto child : children)
        if (child) child->affineMap(matrix);
}
