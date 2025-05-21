#pragma once

#include "SceneObject.h"
#include "PointSet.h"
#include <QMatrix4x4>
#include <QVector3D>
#include <array>

class OctreeNode : public SceneObject {
public:
    OctreeNode(const PointSet& pointSet, int depth = 0, int maxDepth = 3);
    ~OctreeNode();

    void draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const override;
    void affineMap(const QMatrix4x4& matrix) override;

private:
    std::array<OctreeNode*, 8> children;
    PointSet data;
    QVector3D minCorner;
    QVector3D maxCorner;
    int depth;

    void buildChildren(int maxDepth);
};
