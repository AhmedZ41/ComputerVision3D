#pragma once

#include "SceneObject.h"
#include "RenderCamera.h"
#include <QVector4D>
#include <array>

class OctreeNode : public SceneObject {
public:
    OctreeNode(const QVector<QVector4D>& points, const QVector4D& minCorner,
               const QVector4D& maxCorner, int depth = 0);
    ~OctreeNode();

    void draw(const RenderCamera& camera, const QColor& color = Qt::green, float lineWidth = 1.0f) const override;
    void affineMap(const QMatrix4x4& matrix) override;

private:
    QVector4D bboxMin, bboxMax;
    std::array<OctreeNode*, 8> children;
};
