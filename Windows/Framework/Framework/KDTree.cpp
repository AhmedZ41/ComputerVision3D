#include "KDTree.h"
#include <limits>
#include <cmath>
#include <QtMath>
#include <iostream>

KDTree::KDTree() : root(nullptr), maxDepth(0), visualizationLevel(3)
{
    type = SceneObjectType::ST_TREE;
    // Use very small initial bounds that will be replaced when setBounds is called
    boundMin = QVector3D(-0.1f, -0.1f, -0.1f);
    boundMax = QVector3D(0.1f, 0.1f, 0.1f);
}

void KDTree::build(const std::vector<QVector4D> &points)
{
    if (points.empty())
        return;

    // Make a copy of the points to sort them
    std::vector<QVector4D> pointsCopy(points);

    // Build the tree recursively
    root = buildTree(pointsCopy, 0, 0, pointsCopy.size() - 1);

    // Generate visualization after building the tree
    generateVisualization();
}

std::shared_ptr<KDTreeNode> KDTree::buildTree(std::vector<QVector4D> &points, int depth, int start, int end)
{
    if (start > end)
        return nullptr;

    // Update max depth
    if (depth > maxDepth)
        maxDepth = depth;

    // Select axis based on depth (cycle through x, y, z)
    int axis = depth % 3;

    // Sort points along the current axis
    int mid = start + (end - start) / 2;

    // Sort points based on the current axis
    std::nth_element(points.begin() + start,
                     points.begin() + mid,
                     points.begin() + end + 1,
                     [axis](const QVector4D &a, const QVector4D &b)
                     {
                         return a[axis] < b[axis];
                     });

    // Create node and construct subtrees
    auto node = std::make_shared<KDTreeNode>(points[mid], axis);
    node->left = buildTree(points, depth + 1, start, mid - 1);
    node->right = buildTree(points, depth + 1, mid + 1, end);

    return node;
}

float KDTree::distance(const QVector4D &a, const QVector4D &b)
{
    float dx = a.x() - b.x();
    float dy = a.y() - b.y();
    float dz = a.z() - b.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

QVector4D KDTree::findNearest(const QVector4D &query)
{
    if (isEmpty())
        return QVector4D();

    QVector4D nearest;
    float bestDist = std::numeric_limits<float>::max();

    nearestNeighborSearch(query, root, nearest, bestDist);

    return nearest;
}

void KDTree::nearestNeighborSearch(const QVector4D &query, std::shared_ptr<KDTreeNode> node,
                                   QVector4D &nearest, float &bestDist)
{
    if (!node)
        return;

    // Calculate distance to current node
    float dist = distance(query, node->point);

    // Update best distance if current is closer
    if (dist < bestDist)
    {
        bestDist = dist;
        nearest = node->point;
    }

    // Determine which subtree to search first (closer one)
    int axis = node->axis;
    float axisDist = query[axis] - node->point[axis];

    std::shared_ptr<KDTreeNode> firstSearch = (axisDist < 0) ? node->left : node->right;
    std::shared_ptr<KDTreeNode> secondSearch = (axisDist < 0) ? node->right : node->left;

    // Search the closer subtree
    nearestNeighborSearch(query, firstSearch, nearest, bestDist);

    // Check if we need to search the other subtree
    // Only search if the distance to the splitting plane is less than the current best distance
    if (std::abs(axisDist) < bestDist)
    {
        nearestNeighborSearch(query, secondSearch, nearest, bestDist);
    }
}

std::vector<QVector4D> KDTree::findInRange(const QVector4D &query, float radius)
{
    std::vector<QVector4D> result;

    if (isEmpty())
        return result;

    rangeSearch(query, radius, root, result);

    return result;
}

void KDTree::rangeSearch(const QVector4D &query, float radius, std::shared_ptr<KDTreeNode> node,
                         std::vector<QVector4D> &result)
{
    if (!node)
        return;

    // Check if current point is within range
    float dist = distance(query, node->point);
    if (dist <= radius)
    {
        result.push_back(node->point);
    }

    // Check if we need to search left/right subtrees
    int axis = node->axis;
    float axisDist = query[axis] - node->point[axis];

    // If the distance in the current dimension is within radius, search both subtrees
    if (axisDist - radius <= 0)
    {
        rangeSearch(query, radius, node->left, result);
    }

    if (axisDist + radius >= 0)
    {
        rangeSearch(query, radius, node->right, result);
    }
}

void KDTree::setBounds(const QVector3D &min, const QVector3D &max)
{
    boundMin = min;
    boundMax = max;

    // Regenerate visualization with the new bounds
    if (!isEmpty())
    {
        generateVisualization();
    }
}

void KDTree::setVisualizationLevel(int level)
{
    visualizationLevel = level;

    // Regenerate visualization with the new level
    if (!isEmpty())
    {
        generateVisualization();
    }
}

void KDTree::generateVisualization()
{
    // Clear previous visualization
    boxes.clear();

    // Generate boxes for visualization
    if (!isEmpty())
    {
        // Create the root box that encompasses the entire space
        // Use the bounds directly (no further expansion)
        QVector3D rootMin = boundMin;
        QVector3D rootMax = boundMax;

        SpatialPartitionBox rootBox;
        rootBox.center = QVector4D((rootMin.x() + rootMax.x()) * 0.5f,
                                   (rootMin.y() + rootMax.y()) * 0.5f,
                                   (rootMin.z() + rootMax.z()) * 0.5f,
                                   1.0f);
        rootBox.dimensions = rootMax - rootMin;
        rootBox.level = 0;

        // Add the root box
        boxes.push_back(rootBox);

        // Now generate the rest of the boxes for subdivision
        std::vector<QVector4D> allPoints;
        collectAllPoints(root, allPoints);

        generateBoxesRecursive(root, 0, rootMin, rootMax, allPoints);
    }
}

// Helper method to collect all points in the tree
void KDTree::collectAllPoints(std::shared_ptr<KDTreeNode> node, std::vector<QVector4D> &points)
{
    if (!node)
        return;

    points.push_back(node->point);
    collectAllPoints(node->left, points);
    collectAllPoints(node->right, points);
}

// Helper method to collect points in a specific region
void KDTree::collectPointsInRegion(std::shared_ptr<KDTreeNode> node,
                                   const QVector3D &min, const QVector3D &max,
                                   std::vector<QVector4D> &points)
{
    if (!node)
        return;

    // Check if this point is in the region
    bool inRegion = true;
    for (int i = 0; i < 3; i++)
    {
        if (node->point[i] < min[i] || node->point[i] > max[i])
        {
            inRegion = false;
            break;
        }
    }

    if (inRegion)
    {
        points.push_back(node->point);
    }

    // Use the KDTree structure to efficiently traverse - eliminate branches that can't contain points in the region
    int axis = node->axis;

    // Check if left subtree could have points in region (if min[axis] <= splitting value)
    if (node->left && min[axis] <= node->point[axis])
    {
        collectPointsInRegion(node->left, min, max, points);
    }

    // Check if right subtree could have points in region (if max[axis] >= splitting value)
    if (node->right && max[axis] >= node->point[axis])
    {
        collectPointsInRegion(node->right, min, max, points);
    }
}

// Calculate actual bounding box for a set of points
void KDTree::calculatePointsBounds(const std::vector<QVector4D> &points,
                                   QVector3D &outMin, QVector3D &outMax)
{
    if (points.empty())
    {
        outMin = QVector3D(0, 0, 0);
        outMax = QVector3D(0, 0, 0);
        return;
    }

    outMin = QVector3D(std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max());
    outMax = QVector3D(-std::numeric_limits<float>::max(),
                       -std::numeric_limits<float>::max(),
                       -std::numeric_limits<float>::max());

    for (const auto &point : points)
    {
        for (int i = 0; i < 3; i++)
        {
            outMin[i] = std::min(outMin[i], point[i]);
            outMax[i] = std::max(outMax[i], point[i]);
        }
    }

    // Add a buffer to ensure points are enclosed
    float bufferPercent = 0.05f; // 5% buffer for better visibility
    QVector3D dimensions = outMax - outMin;
    QVector3D buffer = dimensions * bufferPercent;

    // Minimum buffer size to ensure very small regions are still visible
    float minBuffer = 0.001f;
    for (int i = 0; i < 3; i++)
    {
        buffer[i] = std::max(buffer[i], minBuffer);
    }

    outMin -= buffer;
    outMax += buffer;
}

void KDTree::generateBoxesRecursive(std::shared_ptr<KDTreeNode> node, int level,
                                    QVector3D min, QVector3D max,
                                    const std::vector<QVector4D> &allPoints)
{
    if (!node || level >= visualizationLevel)
        return;

    // Ensure min is actually smaller than max for all components
    QVector3D actualMin, actualMax;
    for (int i = 0; i < 3; i++)
    {
        actualMin[i] = qMin(min[i], max[i]);
        actualMax[i] = qMax(min[i], max[i]);
    }
    min = actualMin;
    max = actualMax;

    // Get the splitting axis
    int axis = node->axis;

    // Calculate the splitting point
    float splitValue = node->point[axis];

    // Create the left and right child boxes
    QVector3D leftMax = max;
    leftMax[axis] = splitValue;

    QVector3D rightMin = min;
    rightMin[axis] = splitValue;

    // Get points in each partition to calculate actual bounds
    std::vector<QVector4D> leftPoints, rightPoints;
    if (level > 0 && node->left)
    {
        collectPointsInRegion(node->left, min, leftMax, leftPoints);

        if (!leftPoints.empty())
        {
            QVector3D leftPartitionMin, leftPartitionMax;
            calculatePointsBounds(leftPoints, leftPartitionMin, leftPartitionMax);

            // Create left child box with actual bounds
            SpatialPartitionBox leftBox;
            leftBox.center = QVector4D((leftPartitionMin.x() + leftPartitionMax.x()) * 0.5f,
                                       (leftPartitionMin.y() + leftPartitionMax.y()) * 0.5f,
                                       (leftPartitionMin.z() + leftPartitionMax.z()) * 0.5f,
                                       1.0f);
            leftBox.dimensions = leftPartitionMax - leftPartitionMin;
            leftBox.level = level;
            boxes.push_back(leftBox);
        }
    }

    if (level > 0 && node->right)
    {
        collectPointsInRegion(node->right, rightMin, max, rightPoints);

        if (!rightPoints.empty())
        {
            QVector3D rightPartitionMin, rightPartitionMax;
            calculatePointsBounds(rightPoints, rightPartitionMin, rightPartitionMax);

            // Create right child box with actual bounds
            SpatialPartitionBox rightBox;
            rightBox.center = QVector4D((rightPartitionMin.x() + rightPartitionMax.x()) * 0.5f,
                                        (rightPartitionMin.y() + rightPartitionMax.y()) * 0.5f,
                                        (rightPartitionMin.z() + rightPartitionMax.z()) * 0.5f,
                                        1.0f);
            rightBox.dimensions = rightPartitionMax - rightPartitionMin;
            rightBox.level = level;
            boxes.push_back(rightBox);
        }
    }

    // Process left subtree
    if (node->left)
    {
        generateBoxesRecursive(node->left, level + 1, min, leftMax, allPoints);
    }

    // Process right subtree
    if (node->right)
    {
        generateBoxesRecursive(node->right, level + 1, rightMin, max, allPoints);
    }
}

void KDTree::affineMap(const QMatrix4x4 &matrix)
{
    // Transform all box centers (we will recalculate the dimensions in generateVisualization)
    for (auto &box : boxes)
    {
        box.center = matrix * box.center;
    }
}

void KDTree::draw(const RenderCamera &renderer, const QColor &color, float transparency) const
{
    // Make sure we have boxes to draw
    if (boxes.empty())
    {
        return;
    }

    // First sort boxes by level (deeper levels first, so they appear inside outer boxes)
    std::vector<const SpatialPartitionBox *> sortedBoxes;
    for (const auto &box : boxes)
    {
        sortedBoxes.push_back(&box);
    }

    std::sort(sortedBoxes.begin(), sortedBoxes.end(),
              [](const SpatialPartitionBox *a, const SpatialPartitionBox *b)
              {
                  return a->level > b->level;
              });

    // Draw all visualization boxes (inner boxes first)
    for (const auto boxPtr : sortedBoxes)
    {
        const auto &box = *boxPtr;

        // Create the eight corners of the box
        QVector3D center = box.center.toVector3D();
        QVector3D halfDimensions = box.dimensions * 0.5f;

        QVector3D corners[8];
        corners[0] = center + QVector3D(-halfDimensions.x(), -halfDimensions.y(), -halfDimensions.z());
        corners[1] = center + QVector3D(halfDimensions.x(), -halfDimensions.y(), -halfDimensions.z());
        corners[2] = center + QVector3D(halfDimensions.x(), halfDimensions.y(), -halfDimensions.z());
        corners[3] = center + QVector3D(-halfDimensions.x(), halfDimensions.y(), -halfDimensions.z());
        corners[4] = center + QVector3D(-halfDimensions.x(), -halfDimensions.y(), halfDimensions.z());
        corners[5] = center + QVector3D(halfDimensions.x(), -halfDimensions.y(), halfDimensions.z());
        corners[6] = center + QVector3D(halfDimensions.x(), halfDimensions.y(), halfDimensions.z());
        corners[7] = center + QVector3D(-halfDimensions.x(), halfDimensions.y(), halfDimensions.z());

        // Get the line width for this level
        float lineWidth = SpatialTree::getLevelLineWidth(box.level);

        // Get the color for this level
        QColor levelColor = SpatialTree::getLevelColor(box.level);

        // Draw only the wireframe (no filled faces)
        // Bottom face edges
        renderer.renderLine(corners[0], corners[1], levelColor, lineWidth);
        renderer.renderLine(corners[1], corners[2], levelColor, lineWidth);
        renderer.renderLine(corners[2], corners[3], levelColor, lineWidth);
        renderer.renderLine(corners[3], corners[0], levelColor, lineWidth);

        // Top face edges
        renderer.renderLine(corners[4], corners[5], levelColor, lineWidth);
        renderer.renderLine(corners[5], corners[6], levelColor, lineWidth);
        renderer.renderLine(corners[6], corners[7], levelColor, lineWidth);
        renderer.renderLine(corners[7], corners[4], levelColor, lineWidth);

        // Connecting edges
        renderer.renderLine(corners[0], corners[4], levelColor, lineWidth);
        renderer.renderLine(corners[1], corners[5], levelColor, lineWidth);
        renderer.renderLine(corners[2], corners[6], levelColor, lineWidth);
        renderer.renderLine(corners[3], corners[7], levelColor, lineWidth);
    }
}

std::vector<QVector4D> KDTree::getPoints() const
{
    std::vector<QVector4D> points;
    // For simplicity, just return empty vector
    return points;
}
