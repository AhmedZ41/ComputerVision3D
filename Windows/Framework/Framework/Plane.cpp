//
// (c) Georg Umlauf, 2021
//
#include "Plane.h"
#include "GLConvenience.h"
#include "QtConvenience.h"

Plane::Plane(const QVector4D &_origin, const QVector4D &_normal) : origin(_origin),
                                                                   normal(_normal)
{
    type = SceneObjectType::ST_PLANE;
    normal.normalize();
}

void Plane::affineMap(const QMatrix4x4 &M)
{
    origin = M * origin;
    normal = M * normal;
    normal.normalize();
}

Plane &Plane::operator=(const Plane &p)
{
    if (this != &p)
    {
        origin = p.origin;
        normal = p.normal;
    }
    return *this;
}

std::vector<QVector4D> Plane::getPoints() const
{
    // For a plane, we return the origin and points defining the corners of the visualization plane
    std::vector<QVector4D> points;
    points.push_back(origin); // Add the origin point

    // Generate a basis for the plane
    QVector3D n(normal);
    QVector3D y = QVector3D::crossProduct(e1, n);
    if (y.length() < 0.001f)
        y = QVector3D::crossProduct(e2, n);
    QVector3D x = QVector3D::crossProduct(-y, n);
    x.normalize();
    y.normalize();

    // Add points for the four corners of the plane visualization
    float scale = 1.0f;
    points.push_back(QVector4D(QVector3D(origin) + scale * (-x - y), 1.0f));
    points.push_back(QVector4D(QVector3D(origin) + scale * (-x + y), 1.0f));
    points.push_back(QVector4D(QVector3D(origin) + scale * (x + y), 1.0f));
    points.push_back(QVector4D(QVector3D(origin) + scale * (x - y), 1.0f));

    return points;
}

void Plane::draw(const RenderCamera &renderer, const QColor &color, float transparency) const
{
    QVector3D n(normal);
    QVector3D o(origin);
    QVector3D y = QVector3D::crossProduct(e1, n);
    if (y.length() < 0.001f)
        y = QVector3D::crossProduct(e2, n);
    QVector3D x = QVector3D::crossProduct(-y, n);
    x.normalize();
    y.normalize();

    renderer.renderLine(o, o + 0.3f * n, color);

    std::vector<QVector2D> bb;
    bb.push_back(QVector2D(-1, -1));
    bb.push_back(QVector2D(1, 1));
    renderer.renderPlane(o + bb[0][0] * x + bb[0][1] * y,
                         o + bb[0][0] * x + bb[1][1] * y,
                         o + bb[1][0] * x + bb[1][1] * y,
                         o + bb[1][0] * x + bb[0][1] * y,
                         color, transparency);
}
