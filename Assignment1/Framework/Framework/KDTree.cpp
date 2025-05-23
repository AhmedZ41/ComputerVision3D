#include "KDTree.h"

// Constructs a KDTree from a given PointCloud
// Parameters:
// - pointCloud: pointer to the point cloud object
// - maxDepth: maximum depth the KD tree should reach
// - minPoints: minimum number of points in a node before stopping subdivision
KDTree::KDTree(PointCloud* pointCloud, int maxDepth, int minPoints) {
    std::vector<QVector4D> points;

    // Copy all points from the PointCloud into a std::vector
    for (const auto& p : *pointCloud)
        points.push_back(p);

    // Create a PointSet wrapper that also calculates the bounding box
    PointSet set(points);

    // Build the tree recursively starting from depth 0
    root = new KDNode(set, 0, maxDepth, minPoints);
}


// Draws the KDTree using the given render camera
void KDTree::draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const {
    if (root)
        root->draw(renderer, color, lineWidth);  // Delegate drawing to the root node
}


// Applies an affine transformation (e.g., translation, rotation, scaling)
// to all points in the KD tree
void KDTree::affineMap(const QMatrix4x4& matrix) {
    if (root)
        root->affineMap(matrix);  // Transform points and update bounding boxes
}


//destructor
KDTree::~KDTree() {
    delete root;
}
