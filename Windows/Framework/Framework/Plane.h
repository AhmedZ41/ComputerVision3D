//
// (c) Georg Umlauf, 2021
//
#pragma once

#include "SceneObject.h"
#include "Axes.h"

class Plane : public SceneObject
{
private:
    QVector4D origin, normal;

public:
    Plane(const QVector4D &_origin = E1 + E0,
          const QVector4D &_normal = E1);
    virtual ~Plane() override {}

    virtual void affineMap(const QMatrix4x4 &matrix) override;
    virtual void draw(const RenderCamera &renderer,
                      const QColor &color = COLOR_PLANE,
                      float transparency = 0.2f) const override;

    // Implementation of the pure virtual method from SceneObject
    virtual std::vector<QVector4D> getPoints() const override;

    // Getter methods for origin and normal
    QVector4D getOrigin() const { return origin; }
    QVector4D getNormal() const { return normal; }

    Plane &operator=(const Plane &p);
};
