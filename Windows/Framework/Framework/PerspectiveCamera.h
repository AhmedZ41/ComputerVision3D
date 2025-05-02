#pragma once

#ifndef PERSPECTIVECAMERA_H
#define PERSPECTIVECAMERA_H


#include "SceneObject.h"
#include <QVector4D>
#include <QMatrix4x4>

class PerspectiveCamera : public SceneObject {
private:
    QVector4D centerOfProjection; // N
    QVector4D imagePlaneCenter;   // H
    QMatrix4x4 rotation;          // Rotation matrix
    float focalLength;
    float fovY;
    float aspect;

public:
    PerspectiveCamera(const QVector4D& center,
                      const QMatrix4x4& rot,
                      float focalLength,
                      float fovY_degrees,
                      float aspectRatio);

    void setFocalLength(float f);
    void setRotation(const QMatrix4x4& rot);
    void setImagePlaneCenter(const QVector4D& h);

    float getPlaneSize() const;
    float getFocalLength() const;

    virtual void affineMap(const QMatrix4x4& matrix) override;
    virtual void draw(const RenderCamera& renderer, const QColor& color = QColor(255, 0, 0), float lineWidth = 2.0f) const override;
    virtual std::vector<QVector4D> getPoints() const override;

    QVector3D projectPoint(const QVector4D& point) const;
    QVector4D getCenter() const { return centerOfProjection; }
    QMatrix4x4 getRotation() const { return rotation; }

    void drawProjections(const RenderCamera& renderer, const std::vector<SceneObject*>& scene, const QColor& color = Qt::yellow) const;
};

#endif // PERSPECTIVECAMERA_H
