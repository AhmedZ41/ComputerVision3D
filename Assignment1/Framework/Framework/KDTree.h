#ifndef KDTREE_H
#define KDTREE_H

#include "SceneObject.h"
#include "KDNode.h"
#include "PointCloud.h"
#include "PointSet.h"

class KDTree : public SceneObject {
public:
    KDTree(PointCloud* pointCloud, int maxDepth = 3, int minPoints = 10);

    void draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const override;
    void affineMap(const QMatrix4x4& matrix) override;
    SceneObjectType getType() const override { return SceneObjectType::ST_KD_TREE; }

    ~KDTree();

private:
    KDNode* root = nullptr;
};

#endif // KDTREE_H
