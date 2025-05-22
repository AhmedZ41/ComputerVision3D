#ifndef KDTREE_H
#define KDTREE_H

#include <QtGui>
#include <vector>
#include <memory>
#include <algorithm>
#include "SceneObject.h"
#include "Cube.h"
#include "SpatialTree.h"

// KDTree node structure
struct KDTreeNode
{
    QVector4D point;                   // 3D point with homogeneous coordinate
    int axis;                          // Splitting axis (0=x, 1=y, 2=z)
    std::shared_ptr<KDTreeNode> left;  // Left subtree (points with smaller values)
    std::shared_ptr<KDTreeNode> right; // Right subtree (points with larger values)

    KDTreeNode(const QVector4D &p, int a) : point(p), axis(a), left(nullptr), right(nullptr) {}
};

class KDTree : public SpatialTree
{
private:
    std::shared_ptr<KDTreeNode> root;
    int maxDepth;           // Maximum depth of the tree
    int visualizationLevel; // Maximum level to visualize

    // Store bounding box of the model for visualization
    QVector3D boundMin;
    QVector3D boundMax;

    // Store the visualization boxes
    std::vector<SpatialPartitionBox> boxes;

    // Helper methods for tree construction and querying
    std::shared_ptr<KDTreeNode> buildTree(std::vector<QVector4D> &points, int depth, int start, int end);
    void nearestNeighborSearch(const QVector4D &query, std::shared_ptr<KDTreeNode> node,
                               QVector4D &nearest, float &bestDist);
    void rangeSearch(const QVector4D &query, float radius, std::shared_ptr<KDTreeNode> node,
                     std::vector<QVector4D> &result);

    // Utility functions
    float distance(const QVector4D &a, const QVector4D &b);

    // Generate visualization boxes for the spatial partitioning
    void generateVisualization();
    void generateBoxesRecursive(std::shared_ptr<KDTreeNode> node, int level,
                                QVector3D min, QVector3D max,
                                const std::vector<QVector4D> &allPoints);

    // Helper methods for improved visualization
    void collectAllPoints(std::shared_ptr<KDTreeNode> node, std::vector<QVector4D> &points);
    void collectPointsInRegion(std::shared_ptr<KDTreeNode> node,
                               const QVector3D &min, const QVector3D &max,
                               std::vector<QVector4D> &points);
    void calculatePointsBounds(const std::vector<QVector4D> &points,
                               QVector3D &outMin, QVector3D &outMax);

public:
    KDTree();

    // Build KDTree from a point cloud
    void build(const std::vector<QVector4D> &points);

    // Set bounds for visualization
    void setBounds(const QVector3D &min, const QVector3D &max);

    // Set maximum visualization level
    void setVisualizationLevel(int level);

    // Search queries
    QVector4D findNearest(const QVector4D &query);
    std::vector<QVector4D> findInRange(const QVector4D &query, float radius);

    // Check if tree is built
    bool isEmpty() const { return root == nullptr; }

    // Get the root node - for visualization
    std::shared_ptr<KDTreeNode> getRoot() const { return root; }

    // SceneObject interface implementation
    virtual void affineMap(const QMatrix4x4 &matrix) override;
    virtual void draw(const RenderCamera &renderer,
                      const QColor &color = COLOR_TREE,
                      float transparency = 0.4f) const override;
    virtual std::vector<QVector4D> getPoints() const override;
};

#endif // KDTREE_H
