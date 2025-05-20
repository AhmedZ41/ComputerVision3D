#ifndef KDNODE_H
#define KDNODE_H

#include "PointSet.h"
#include "SceneObject.h"


class KDNode : public SceneObject {
public:
    // Constructor
    KDNode(const PointSet& pointSet, int depth = 0, int maxDepth = 3, int minPoints = 10);

    // Tree structure
    KDNode* left  = nullptr;
    KDNode* right = nullptr;

    // Data
    PointSet data;         // current node's point set
    int axis = -1;         // 0 = x, 1 = y, 2 = z
    float splitValue = 0;  // coordinate value of the split

    // Draw function to be implemented later
    // This overrides the SceneObject interface
    void draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const override;

    // Internal recursive draw with depth tracking (not part of override)
    void draw(const RenderCamera& renderer, const QColor& color, float lineWidth, int currentDepth) const;
    SceneObjectType getType() const override { return SceneObjectType::ST_KD_TREE; }
    void affineMap(const QMatrix4x4& matrix) override;



    ~KDNode(); // destructor to clean up
};

#endif // KDNODE_H
