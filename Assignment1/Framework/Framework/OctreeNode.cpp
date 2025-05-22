#include "OctreeNode.h"

// Constructor: Builds the octree recursively from input points and bounding box.
// - points: all points inside the current node
// - minCorner, maxCorner: bounding box corners for this node
// - depth: current depth of this node in the tree
OctreeNode::OctreeNode(const QVector<QVector4D>& points,
                       const QVector4D& minCorner,
                       const QVector4D& maxCorner,
                       int depth)
{
    // Set the scene object type so that SceneManager can recognize it
    type = SceneObjectType::ST_CUBE;

    // Store bounding box
    bboxMin = minCorner;
    bboxMax = maxCorner;

    // Initialize all 8 child pointers to nullptr
    children.fill(nullptr);

    // Stopping condition: stop splitting if max depth reached or too few points
    if (depth >= 3 || points.size() < 20)
        return;

    // Compute center of bounding box to subdivide it
    QVector4D mid = 0.5f * (bboxMin + bboxMax);

    // Step 1: Distribute points into one of 8 children based on their position
    QVector<QVector4D> childPoints[8];
    for (const auto& p : points) {
        int index = (p.x() > mid.x())      // 1 → right
                    + 2 * (p.y() > mid.y())  // 2 → top
                    + 4 * (p.z() > mid.z()); // 4 → front
        childPoints[index].push_back(p);
    }

    // Step 2: For each child region, compute its bounding box and recurse
    for (int i = 0; i < 8; ++i) {
        QVector4D childMin = bboxMin;
        QVector4D childMax = mid;

        if (i & 1) { childMin.setX(mid.x()); childMax.setX(bboxMax.x()); }
        if (i & 2) { childMin.setY(mid.y()); childMax.setY(bboxMax.y()); }
        if (i & 4) { childMin.setZ(mid.z()); childMax.setZ(bboxMax.z()); }

        // Only create a child node if it contains points
        if (!childPoints[i].isEmpty())
            children[i] = new OctreeNode(childPoints[i], childMin, childMax, depth + 1);
    }
}


OctreeNode::~OctreeNode() {
    for (auto* child : children)
        delete child;
}

// Draws the bounding box of the node and recursively its children
// Colors change based on depth (X, Y, Z splits)
void OctreeNode::draw(const RenderCamera& camera, const QColor& baseColor, float lineWidth) const {
    // Static variable to track recursion depth per draw call
    static int currentDepth = 0;

    // Set color based on which axis would be split at this level
    QColor color;
    switch (currentDepth % 3) {
    case 0: color = QColor(255, 0, 0); break;   // Red → X-split
    case 1: color = QColor(0, 255, 0); break;   // Green → Y-split
    case 2: color = QColor(0, 0, 255); break;   // Blue → Z-split
    }

    // Draw this node’s bounding box
    camera.renderWireCube(bboxMin.toVector3D(), bboxMax.toVector3D(), color, lineWidth);

    // Recursively draw all children
    currentDepth++;
    for (const auto* child : children) {
        if (child)
            child->draw(camera, color, lineWidth);
    }
    currentDepth--;
}



void OctreeNode::affineMap(const QMatrix4x4&) {
}
