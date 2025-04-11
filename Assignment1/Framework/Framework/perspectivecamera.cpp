#include "PerspectiveCamera.h"
#include "GLConvenience.h"
#include "QtConvenience.h"
#include "Cube.h"  // Access Cube::getPoints()
#include "Plane.h"

#include <cmath>

//
// ====================== Constructor ======================
// Assignment 2 – Define Camera Parameters & Setup
//
PerspectiveCamera::PerspectiveCamera(const QVector4D& position,
                                     const QVector3D& lookDir,
                                     const QVector3D& up,
                                     float f,
                                     float width,
                                     float height)
    : centerOfProjection(position),
    viewDirection(lookDir.normalized()),
    upVector(up.normalized()),
    focalLength(f),
    imagePlaneWidth(width),
    imagePlaneHeight(height)
{
    type = SceneObjectType::ST_PERSPECTIVE_CAMERA;

    computeCameraCoordinateSystem(); // part 2

    // Compute principal point H = N + f * (-v)
    QVector3D imagePlaneCenter = QVector3D(centerOfProjection) + (-viewDirection) * focalLength;
    principalPoint = to4D(imagePlaneCenter);

    // Axes at camera origin (p 2 - Visualize local axes)
    localAxes = new Axes(centerOfProjection, cameraToWorld);

    // Compute corners of image plane
    QVector3D x = rightVector * (imagePlaneWidth / 2.0f);
    QVector3D y = upVector    * (imagePlaneHeight / 2.0f);
    QVector3D tl = imagePlaneCenter - x + y;
    QVector3D tr = imagePlaneCenter + x + y;
    QVector3D br = imagePlaneCenter + x - y;
    QVector3D bl = imagePlaneCenter - x - y;

    //imagePlane = new Plane(to4D(tl), to4D(viewDirection)); // p 2
    imagePlane = new Plane(principalPoint, to4D(viewDirection)); // correct: origin = H (principal point)

}

//
// ====================== Destructor ======================
//
PerspectiveCamera::~PerspectiveCamera()
{
    delete localAxes;
    delete imagePlane;
}

//
// ====================== Draw Method ======================
// p 2 – Visualize camera & image plane
// p 3 – Visualize projections
//
void PerspectiveCamera::draw(const RenderCamera& renderer,
                             const QColor& color,
                             float lineWidth) const
{
    // --- p 2: Camera visualization ---
    localAxes->draw(renderer, QColor(255, 255, 0), lineWidth);                    // Local axes at N
    renderer.renderPoint(centerOfProjection, color, 5.0f);          // Center of projection (N)
    renderer.renderPoint(principalPoint, QColor(255, 0, 255), 5.0f); // Principal point (H)
    renderer.renderLine(centerOfProjection, principalPoint, QColor(255, 0, 0), 2.0f); // Line N → H

    // Image plane quad
    //QVector3D x = rightVector * (imagePlaneWidth / 2.0f);
    //QVector3D y = upVector    * (imagePlaneHeight / 2.0f);
    //QVector3D center = QVector3D(principalPoint);
    //QVector3D tl = center - x + y;
    //QVector3D tr = center + x + y;
    //QVector3D br = center + x - y;
    //QVector3D bl = center - x - y;

    //renderer.renderPlane(tl, tr, br, bl, QColor(200, 0, 100), 0.3f); // Transparent image plane

    imagePlane->draw(renderer, QColor(200, 200, 0), 0.3f);


    // --- p 3: Projected cubes
    std::array<std::array<int, 2>, 12> edges = {{
        {0,1}, {1,2}, {2,3}, {3,0},
        {4,5}, {5,6}, {6,7}, {7,4},
        {0,4}, {1,5}, {2,6}, {3,7}
    }};

    for (const auto& cube : projectedObjects) {
        for (const auto& edge : edges) {
            renderer.renderLine(cube[edge[0]], cube[edge[1]], QColor(255, 0, 0), 1.5f); // Green projected edges
            // Convert the 3D projected point into the image plane's local coordinate system
            /*
            auto projectToImagePlane = [&](const QVector4D& p) -> QVector2D {
                QVector3D vec = QVector3D(p) - QVector3D(principalPoint);
                float u = QVector3D::dotProduct(vec, rightVector);
                float v = QVector3D::dotProduct(vec, upVector);
                return QVector2D(u, v);
            };

            for (const auto& cube : projectedObjects) {
                for (const auto& edge : edges) {
                    QVector4D p1 = cube[edge[0]];
                    QVector4D p2 = cube[edge[1]];

                    QVector2D uv1 = projectToImagePlane(p1);
                    QVector2D uv2 = projectToImagePlane(p2);

                    float w = imagePlaneWidth  / 2.0f;
                    float h = imagePlaneHeight / 2.0f;

                    // Check if at least one of the points is inside the image bounds
                    auto isInside = [&](const QVector2D& uv) {
                        return uv.x() >= -w && uv.x() <= w && uv.y() >= -h && uv.y() <= h;
                    };

                    if (isInside(uv1) || isInside(uv2)) {
                        renderer.renderLine(p1, p2, QColor(0, 255, 0), 1.5f);
                    }
                }
            }*/

        }
    }
}

//
// ====================== Affine Transform ======================
// p 2 – Allow transforming camera with matrix
//
void PerspectiveCamera::affineMap(const QMatrix4x4& matrix)
{
    centerOfProjection = matrix * centerOfProjection;
    principalPoint = matrix * principalPoint;

    QMatrix4x4 R;
    R.setColumn(0, to4D(rightVector));
    R.setColumn(1, to4D(upVector));
    R.setColumn(2, to4D(-viewDirection));
    R.setColumn(3, QVector4D(0, 0, 0, 1));

    cameraToWorld = matrix * R;

    localAxes->affineMap(matrix);
}

//
// ====================== Camera Basis Setup ======================
// p 2 – Compute right, up, forward directions
//
void PerspectiveCamera::computeCameraCoordinateSystem()
{
    rightVector = QVector3D::crossProduct(upVector, viewDirection).normalized();
    upVector    = QVector3D::crossProduct(viewDirection, rightVector).normalized();

    QMatrix4x4 R;
    R.setColumn(0, to4D(rightVector));
    R.setColumn(1, to4D(upVector));
    R.setColumn(2, to4D(-viewDirection));
    R.setColumn(3, QVector4D(0, 0, 0, 1));

    QMatrix4x4 T;
    T.setToIdentity();
    T.translate(QVector3D(centerOfProjection));

    cameraToWorld = T * R;
}

//
// ====================== Cube Projection ======================
// p 3 – Project cube vertices to image plane
//
std::optional<std::array<QVector4D, 8>> PerspectiveCamera::projectCube(const std::array<QVector4D, 8>& cubePoints)
{
    std::array<QVector4D, 8> projected;

    for (int i = 0; i < 8; ++i) {
        QVector4D P = cubePoints[i];
        QVector4D v = P - centerOfProjection;
        float denom = QVector3D::dotProduct(QVector3D(viewDirection), QVector3D(v));

        if (std::abs(denom) < 1e-6f)
            return std::nullopt; // Skip if denominator ≈ 0
        //If denom is very close to zero, it means the ray from the camera to the point is almost parallel to the image plane.


        //lambda is a scalar used to compute the intersection point of a ray (from the camera to a 3D point) with the image plane.
        float lambda = -QVector3D::dotProduct(QVector3D(viewDirection),
                                              QVector3D(centerOfProjection - principalPoint)) / denom;

        projected[i] = centerOfProjection + lambda * v;
    }

    return projected;
}

//
// ====================== Add Cube to Projection ======================
// p 3 – Store projected cube
//
void PerspectiveCamera::addCube(const Cube& cube)
{
    auto cubePoints = cube.getPoints();
    auto projection = projectCube(cubePoints);

    if (projection.has_value())
        projectedObjects.push_back(projection.value());
}
