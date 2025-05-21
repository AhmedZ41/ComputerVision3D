#ifndef OCTREENODE_H
#define OCTREENODE_H

#include <QVector4D>
#include <QVector>
#include <QMatrix4x4>
#include "SceneObject.h"
#include "RenderCamera.h"

class OctreeNode : public SceneObject {
public:
    QVector<QVector4D> points;
    QVector4D bboxMin, bboxMax;  // bounding box of this node
    OctreeNode* children[8] = {nullptr};

    OctreeNode(const QVector<QVector4D>& pts, const QVector4D& minCorner, const QVector4D& maxCorner, int depth = 0);
    ~OctreeNode();

    void draw(const RenderCamera& camera, const QColor& color, float lineWidth) const override;
    void affineMap(const QMatrix4x4& matrix) override;
    SceneObjectType getType() const override { return SceneObjectType::ST_HEXAHEDRON; } // placeholder type
};

#endif // OCTREENODE_H
