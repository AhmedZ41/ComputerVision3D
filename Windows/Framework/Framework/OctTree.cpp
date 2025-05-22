#include "OctTree.h"
#include <cmath>
#include <limits>
#include <algorithm>

OctTree::OctTree() : root(nullptr), maxLevel(8), visualizationLevel(3) {}

void OctTree::build(const std::vector<QVector4D> &points)
{
    type = SceneObjectType::ST_TREE;
    allPoints = points;
    if (points.empty())
        return;
    root = buildTree(points, 0, boundMin, boundMax);
    generateVisualization();
}

std::shared_ptr<OctTreeNode> OctTree::buildTree(const std::vector<QVector4D> &points, int level, QVector3D min, QVector3D max)
{
    if (points.empty() || level > maxLevel)
        return nullptr;
    // If only one point or max level reached, create a leaf
    if (points.size() == 1 || level == maxLevel)
    {
        return std::make_shared<OctTreeNode>(points[0], level);
    }
    // Compute center
    QVector3D center = (min + max) * 0.5f;
    std::vector<QVector4D> octantPoints[8];
    for (const auto &p : points)
    {
        int idx = 0;
        if (p.x() > center.x())
            idx |= 1;
        if (p.y() > center.y())
            idx |= 2;
        if (p.z() > center.z())
            idx |= 4;
        octantPoints[idx].push_back(p);
    }
    auto node = std::make_shared<OctTreeNode>(QVector4D(center, 1.0f), level);
    for (int i = 0; i < 8; ++i)
    {
        QVector3D childMin = min;
        QVector3D childMax = center;
        if (i & 1)
        {
            childMin.setX(center.x());
            childMax.setX(max.x());
        }
        if (i & 2)
        {
            childMin.setY(center.y());
            childMax.setY(max.y());
        }
        if (i & 4)
        {
            childMin.setZ(center.z());
            childMax.setZ(max.z());
        }
        node->children[i] = buildTree(octantPoints[i], level + 1, childMin, childMax);
    }
    return node;
}

void OctTree::setBounds(const QVector3D &min, const QVector3D &max)
{
    boundMin = min;
    boundMax = max;
    if (!isEmpty())
        generateVisualization();
}

void OctTree::setVisualizationLevel(int level)
{
    visualizationLevel = level;
    if (!isEmpty())
        generateVisualization();
}

QVector4D OctTree::findNearest(const QVector4D &query)
{
    QVector4D nearest;
    float bestDist = std::numeric_limits<float>::max();
    std::function<void(std::shared_ptr<OctTreeNode>, QVector3D, QVector3D)> search = [&](std::shared_ptr<OctTreeNode> node, QVector3D min, QVector3D max)
    {
        if (!node)
            return;
        float dist = (query.toVector3D() - node->point.toVector3D()).length();
        if (dist < bestDist)
        {
            bestDist = dist;
            nearest = node->point;
        }
        if (node->children.empty())
            return;
        QVector3D center = (min + max) * 0.5f;
        for (int i = 0; i < 8; ++i)
        {
            if (node->children[i])
            {
                QVector3D childMin = min, childMax = center;
                if (i & 1)
                {
                    childMin.setX(center.x());
                    childMax.setX(max.x());
                }
                if (i & 2)
                {
                    childMin.setY(center.y());
                    childMax.setY(max.y());
                }
                if (i & 4)
                {
                    childMin.setZ(center.z());
                    childMax.setZ(max.z());
                }
                search(node->children[i], childMin, childMax);
            }
        }
    };
    search(root, boundMin, boundMax);
    return nearest;
}

std::vector<QVector4D> OctTree::findInRange(const QVector4D &query, float radius)
{
    std::vector<QVector4D> result;
    std::function<void(std::shared_ptr<OctTreeNode>, QVector3D, QVector3D)> search = [&](std::shared_ptr<OctTreeNode> node, QVector3D min, QVector3D max)
    {
        if (!node)
            return;
        float dist = (query.toVector3D() - node->point.toVector3D()).length();
        if (dist <= radius)
            result.push_back(node->point);
        if (node->children.empty())
            return;
        QVector3D center = (min + max) * 0.5f;
        for (int i = 0; i < 8; ++i)
        {
            if (node->children[i])
            {
                QVector3D childMin = min, childMax = center;
                if (i & 1)
                {
                    childMin.setX(center.x());
                    childMax.setX(max.x());
                }
                if (i & 2)
                {
                    childMin.setY(center.y());
                    childMax.setY(max.y());
                }
                if (i & 4)
                {
                    childMin.setZ(center.z());
                    childMax.setZ(max.z());
                }
                // Simple AABB check
                QVector3D q = query.toVector3D();
                QVector3D closest = q;
                for (int d = 0; d < 3; ++d)
                {
                    if (q[d] < childMin[d])
                        closest[d] = childMin[d];
                    else if (q[d] > childMax[d])
                        closest[d] = childMax[d];
                }
                if ((q - closest).length() <= radius)
                    search(node->children[i], childMin, childMax);
            }
        }
    };
    search(root, boundMin, boundMax);
    return result;
}

bool OctTree::isEmpty() const { return !root; }

void OctTree::collectAllPoints(std::shared_ptr<OctTreeNode> node, std::vector<QVector4D> &points) const
{
    if (!node)
        return;
    points.push_back(node->point);
    for (const auto &child : node->children)
    {
        if (child)
            collectAllPoints(child, points);
    }
}

void OctTree::generateVisualization()
{
    boxes.clear();
    if (!isEmpty())
    {
        generateBoxesRecursive(root, 0, boundMin, boundMax);
    }
}

void OctTree::generateBoxesRecursive(std::shared_ptr<OctTreeNode> node, int level, QVector3D min, QVector3D max)
{
    if (!node || level > visualizationLevel)
        return;
    SpatialPartitionBox box;
    QVector3D center = (min + max) * 0.5f;
    box.center = QVector4D(center, 1.0f);
    box.dimensions = max - min;
    box.level = level;
    boxes.push_back(box);
    QVector3D childMin, childMax;
    for (int i = 0; i < 8; ++i)
    {
        childMin = min;
        childMax = center;
        if (i & 1)
        {
            childMin.setX(center.x());
            childMax.setX(max.x());
        }
        if (i & 2)
        {
            childMin.setY(center.y());
            childMax.setY(max.y());
        }
        if (i & 4)
        {
            childMin.setZ(center.z());
            childMax.setZ(max.z());
        }
        if (node->children[i])
            generateBoxesRecursive(node->children[i], level + 1, childMin, childMax);
    }
}

void OctTree::affineMap(const QMatrix4x4 &matrix)
{
    for (auto &box : boxes)
    {
        box.center = matrix * box.center;
    }
}

void OctTree::draw(const RenderCamera &renderer, const QColor &color, float transparency) const
{
    if (boxes.empty())
        return;
    std::vector<const SpatialPartitionBox *> sortedBoxes;
    for (const auto &box : boxes)
        sortedBoxes.push_back(&box);
    std::sort(sortedBoxes.begin(), sortedBoxes.end(), [](const SpatialPartitionBox *a, const SpatialPartitionBox *b)
              { return a->level > b->level; });
    for (const auto boxPtr : sortedBoxes)
    {
        const auto &box = *boxPtr;
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
        float lineWidth = SpatialTree::getLevelLineWidth(box.level);
        QColor levelColor = SpatialTree::getLevelColor(box.level);
        renderer.renderLine(corners[0], corners[1], levelColor, lineWidth);
        renderer.renderLine(corners[1], corners[2], levelColor, lineWidth);
        renderer.renderLine(corners[2], corners[3], levelColor, lineWidth);
        renderer.renderLine(corners[3], corners[0], levelColor, lineWidth);
        renderer.renderLine(corners[4], corners[5], levelColor, lineWidth);
        renderer.renderLine(corners[5], corners[6], levelColor, lineWidth);
        renderer.renderLine(corners[6], corners[7], levelColor, lineWidth);
        renderer.renderLine(corners[7], corners[4], levelColor, lineWidth);
        renderer.renderLine(corners[0], corners[4], levelColor, lineWidth);
        renderer.renderLine(corners[1], corners[5], levelColor, lineWidth);
        renderer.renderLine(corners[2], corners[6], levelColor, lineWidth);
        renderer.renderLine(corners[3], corners[7], levelColor, lineWidth);
    }
}

std::vector<QVector4D> OctTree::getPoints() const
{
    std::vector<QVector4D> points;
    collectAllPoints(root, points);
    return points;
}
