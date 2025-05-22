#include "KDTree.h"

KDTree::KDTree(PointCloud* pointCloud, int maxDepth, int minPoints) {
    std::vector<QVector4D> points;
    for (const auto& p : *pointCloud)
        points.push_back(p);
    PointSet set(points);
    root = new KDNode(set, 0, maxDepth, minPoints);
}

void KDTree::draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const {
    if (root) root->draw(renderer, color, lineWidth);
}

void KDTree::affineMap(const QMatrix4x4& matrix) {
    if (root) root->affineMap(matrix);
}

KDTree::~KDTree() {
    delete root;
}
