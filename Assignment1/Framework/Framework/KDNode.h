#ifndef KDNODE_H
#define KDNODE_H

#include "PointSet.h"

class KDNode {
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
    void draw(const RenderCamera& renderer, const QColor& color, float lineWidth, int currentDepth = 0) const;

    ~KDNode(); // destructor to clean up
};

#endif // KDNODE_H
