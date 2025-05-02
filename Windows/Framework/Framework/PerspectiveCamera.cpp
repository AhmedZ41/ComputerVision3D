#include "PerspectiveCamera.h"
#include "Axes.h"
#include "Hexahedron.h"

PerspectiveCamera::PerspectiveCamera(const QVector4D& center,
                                     const QMatrix4x4& rot,
                                     float f,
                                     float fovY_degrees,
                                     float aspectRatio)
    : centerOfProjection(center),
    rotation(rot),
    focalLength(f),
    fovY(fovY_degrees),
    aspect(aspectRatio)
{
    QVector3D forward = -rotation.column(2).toVector3D().normalized();
    imagePlaneCenter = center + QVector4D(forward * f, 0.0f);
    type = SceneObjectType::ST_PERSPECTIVE_CAMERA;
}

void PerspectiveCamera::setFocalLength(float f) {
    focalLength = f;
    QVector3D forward = -rotation.column(2).toVector3D().normalized();
    imagePlaneCenter = centerOfProjection + QVector4D(forward * focalLength, 0.0f);
}

void PerspectiveCamera::setRotation(const QMatrix4x4& rot) {
    rotation = rot;
    QVector3D forward = -rotation.column(2).toVector3D().normalized();
    imagePlaneCenter = centerOfProjection + QVector4D(forward * focalLength, 0.0f);
}

void PerspectiveCamera::setImagePlaneCenter(const QVector4D& h) {
    imagePlaneCenter = h;
}

float PerspectiveCamera::getPlaneSize() const {
    float imagePlaneHeight = 2.0f * focalLength * std::tan(fovY * 0.5f * M_PI / 180.0f);
    float imagePlaneWidth = imagePlaneHeight * aspect;
    return imagePlaneWidth / 2.0f; // half-width is used for clipping in u-direction
}

float PerspectiveCamera::getFocalLength() const {
    return focalLength;
}

std::vector<QVector4D> PerspectiveCamera::getPoints() const {
    return {};  // not used for projection
}


void PerspectiveCamera::affineMap(const QMatrix4x4& matrix) {
    centerOfProjection = matrix * centerOfProjection;
    imagePlaneCenter = matrix * imagePlaneCenter;
    rotation = matrix * rotation;
}

void PerspectiveCamera::draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const {
    // Draw center of projection
    renderer.renderPoint(centerOfProjection, color, 5.0f);

    // Draw image plane as square (simplified)
    QVector3D right = rotation.column(0).toVector3D().normalized();
    QVector3D up = rotation.column(1).toVector3D().normalized();
    QVector3D center = imagePlaneCenter.toVector3D();

    float imagePlaneHeight = 2 * focalLength * tan(fovY * 0.5 * M_PI / 180.0f);
    float imagePlaneWidth  = imagePlaneHeight * aspect;
    float planeSize        = imagePlaneWidth / 2.0f;

    QVector3D tl = center + planeSize * (-right + up);
    QVector3D tr = center + planeSize * (right + up);
    QVector3D br = center + planeSize * (right - up);
    QVector3D bl = center + planeSize * (-right - up);

    renderer.renderPlane(tl, tr, br, bl, color, 0.2f);

    // Draw viewing rays
    renderer.renderLine(centerOfProjection.toVector3D(), tl, color, lineWidth);
    renderer.renderLine(centerOfProjection.toVector3D(), tr, color, lineWidth);
    renderer.renderLine(centerOfProjection.toVector3D(), br, color, lineWidth);
    renderer.renderLine(centerOfProjection.toVector3D(), bl, color, lineWidth);

    // Draw camera axes
    //Axes axes(centerOfProjection, rotation);
    //axes.draw(renderer, color, lineWidth);
}

QVector3D PerspectiveCamera::projectPoint(const QVector4D& point) const {
    QVector3D dir = point.toVector3D() - centerOfProjection.toVector3D();
    QVector3D forward = -rotation.column(2).toVector3D().normalized();
    float t = focalLength / QVector3D::dotProduct(dir, forward);
    return centerOfProjection.toVector3D() + t * dir;
}


void PerspectiveCamera::drawProjections(const RenderCamera& renderer, const std::vector<SceneObject*>& scene, const QColor& color) const {
    QVector3D cx = rotation.column(0).toVector3D().normalized();  // right
    QVector3D cy = rotation.column(1).toVector3D().normalized();  // up
    QVector3D cz = -rotation.column(2).toVector3D().normalized(); // forward
    QVector3D N = centerOfProjection.toVector3D();
    QVector3D H = imagePlaneCenter.toVector3D();

    // dynamic plane size (half width in u-direction)
    float planeHalfWidth = getPlaneSize();
    float planeHalfHeight = planeHalfWidth / aspect;  // because width = height * aspect

    for (const auto* obj : scene) {
        if (!obj || obj == this || obj->getType() == SceneObjectType::ST_AXES)
            continue;

        auto hex = dynamic_cast<const Hexahedron*>(obj);
        if (!hex) continue;

        std::vector<QVector4D> points = hex->getPoints();
        std::vector<std::pair<int, int>> edges = hex->getEdgeIndices();
        std::vector<QVector3D> projected;

        for (const auto& p4 : points) {
            QVector3D P = p4.toVector3D();
            QVector3D dir = P - N;
            float t = focalLength / QVector3D::dotProduct(dir, cz);
            QVector3D Q = N + t * dir;
            projected.push_back(Q);
        }

        for (const auto& [i0, i1] : edges) {
            QVector3D Q0 = projected[i0];
            QVector3D Q1 = projected[i1];

            QVector2D uv0(QVector3D::dotProduct(Q0 - H, cx), QVector3D::dotProduct(Q0 - H, cy));
            QVector2D uv1(QVector3D::dotProduct(Q1 - H, cx), QVector3D::dotProduct(Q1 - H, cy));
            QVector2D d = uv1 - uv0;

            float t0 = 0.0f, t1 = 1.0f;
            auto clip = [&](float p, float q) -> bool {
                if (p == 0.0f) return q >= 0;
                float r = q / p;
                if (p < 0) {
                    if (r > t1) return false;
                    if (r > t0) t0 = r;
                } else {
                    if (r < t0) return false;
                    if (r < t1) t1 = r;
                }
                return true;
            };

            // Liang–Barsky Clipping in (u, v) Space
            // Reference: https://en.wikipedia.org/wiki/Liang%E2%80%93Barsky_algorithm
            // Clip against all four edges of the image plane in (u,v)
            if (
                clip(-d.x(), uv0.x() + planeHalfWidth) &&
                clip(d.x(), planeHalfWidth - uv0.x()) &&
                clip(-d.y(), uv0.y() + planeHalfHeight) &&
                clip(d.y(), planeHalfHeight - uv0.y())
                ) {
                QVector2D uvA = uv0 + t0 * d;
                QVector2D uvB = uv0 + t1 * d;
                QVector3D worldA = H + uvA.x() * cx + uvA.y() * cy;
                QVector3D worldB = H + uvB.x() * cx + uvB.y() * cy;
                renderer.renderLine(worldA, worldB, color, 1.5f);
            }
        }
    }
}








