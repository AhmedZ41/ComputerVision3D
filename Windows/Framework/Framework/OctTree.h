#ifndef OCTTREE_H
#define OCTTREE_H

#include "SpatialTree.h"
#include <memory>
#include <vector>
#include <QVector3D>
#include <QVector4D>
#include <QColor>

struct OctTreeNode
{
    QVector4D point;
    std::vector<std::shared_ptr<OctTreeNode>> children;
    int level;
    OctTreeNode(const QVector4D &p, int l) : point(p), level(l), children(8, nullptr) {}
};

class OctTree : public SpatialTree
{
private:
    std::shared_ptr<OctTreeNode> root;
    QVector3D boundMin, boundMax;
    int maxLevel;
    int visualizationLevel;
    std::vector<QVector4D> allPoints;
    std::vector<SpatialPartitionBox> boxes;

    std::shared_ptr<OctTreeNode> buildTree(const std::vector<QVector4D> &points, int level, QVector3D min, QVector3D max);
    void collectAllPoints(std::shared_ptr<OctTreeNode> node, std::vector<QVector4D> &points) const;
    void generateVisualization();
    void generateBoxesRecursive(std::shared_ptr<OctTreeNode> node, int level, QVector3D min, QVector3D max);

public:
    OctTree();
    void build(const std::vector<QVector4D> &points) override;
    void setBounds(const QVector3D &min, const QVector3D &max) override;
    void setVisualizationLevel(int level) override;
    QVector4D findNearest(const QVector4D &query) override;
    std::vector<QVector4D> findInRange(const QVector4D &query, float radius) override;
    bool isEmpty() const override;
    void affineMap(const QMatrix4x4 &matrix) override;
    void draw(const RenderCamera &renderer, const QColor &color = QColor(0, 200, 255), float transparency = 0.4f) const override;
    std::vector<QVector4D> getPoints() const override;
};

#endif // OCTTREE_H