#include "PerspectiveCamera.h"
#include "GLConvenience.h"
#include "QtConvenience.h"
#include "Cube.h"
#include "Plane.h"
#include "StereoCamera.h"
#include <cmath>

//
// ========== Constructor ==========
// Initializes camera parameters and sets up the image plane and local axes
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

    computeCameraCoordinateSystem(); // Compute view, up, and right axes

    // Compute principal point: H = N + f * (-v)
    QVector3D imagePlaneCenter = QVector3D(centerOfProjection) + (-viewDirection) * focalLength;
    principalPoint = to4D(imagePlaneCenter);

    // Build local coordinate axes object for visualization
    QMatrix4x4 R;
    R.setColumn(0, to4D(rightVector));
    R.setColumn(1, to4D(upVector));
    R.setColumn(2, to4D(-viewDirection));
    R.setColumn(3, QVector4D(0, 0, 0, 1));
    localAxes = new Axes(centerOfProjection, R);

    // Build image plane (origin at principal point)
    imagePlane = new Plane(principalPoint, to4D(viewDirection));
}

//
// ========== Destructor ==========
// Frees allocated memory for axes and image plane
//
PerspectiveCamera::~PerspectiveCamera()
{
    delete localAxes;
    delete imagePlane;
}

//
// ========== draw() ==========
// Renders the camera's components and its projected cubes
//
void PerspectiveCamera::draw(const RenderCamera& renderer,
                             const QColor& color,
                             float lineWidth) const
{
    // --- Camera visuals: position, principal point, axes
    localAxes->draw(renderer, QColor(255, 255, 0), lineWidth);
    renderer.renderPoint(centerOfProjection, color, 5.0f);
    renderer.renderPoint(principalPoint, QColor(255, 0, 255), 5.0f);
    renderer.renderLine(centerOfProjection, principalPoint, QColor(255, 0, 0), 2.0f);

    // --- Draw image plane
    imagePlane->draw(renderer, QColor(200, 200, 0), 0.3f);

    // --- Draw edges of projected cubes (in red)
    std::array<std::array<int, 2>, 12> edges = {{
        {0,1}, {1,2}, {2,3}, {3,0},
        {4,5}, {5,6}, {6,7}, {7,4},
        {0,4}, {1,5}, {2,6}, {3,7}
    }};
    for (const auto& cube : projectedObjects) {
        for (const auto& edge : edges) {
            renderer.renderLine(cube[edge[0]], cube[edge[1]], QColor(255, 0, 0), 1.5f);
        }
    }

    // --- Draw projection rays from camera to projected points (in blue)
    for (const auto& cube : projectedObjects) {
        for (int i = 0; i < 8; ++i) {
            renderer.renderLine(centerOfProjection, cube[i], QColor(0, 0, 255), 1.0f);
        }
    }
}

//
// ========== affineMap() ==========
// Applies an affine transformation to the camera and updates internal states
//
void PerspectiveCamera::affineMap(const QMatrix4x4& M)
{
    viewDirection = (M * QVector4D(viewDirection, 0.0f)).toVector3D().normalized();
    upVector      = (M * QVector4D(upVector,      0.0f)).toVector3D().normalized();

    computeCameraCoordinateSystem();

    centerOfProjection = M * centerOfProjection;
    principalPoint = to4D(QVector3D(centerOfProjection) + (-viewDirection) * focalLength);

    QMatrix4x4 R;
    R.setColumn(0, to4D(rightVector));
    R.setColumn(1, to4D(upVector));
    R.setColumn(2, to4D(-viewDirection));
    R.setColumn(3, QVector4D(0,0,0,1));
    cameraToWorld = M * R;

    localAxes->affineMap(M);

    clearProjectedCubes(); // Invalidate cached projections
}

//
// ========== computeCameraCoordinateSystem() ==========
// Recalculates the orthonormal view basis (right, up, forward)
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
// ========== projectCube() ==========
// Projects each point of a cube onto the camera's image plane
//
std::optional<std::array<QVector4D, 8>> PerspectiveCamera::projectCube(const std::array<QVector4D, 8>& cubePoints)
{
    std::array<QVector4D, 8> projected;

    for (int i = 0; i < 8; ++i) {
        QVector4D P = cubePoints[i];
        QVector4D v = P - centerOfProjection;
        float denom = QVector3D::dotProduct(QVector3D(viewDirection), QVector3D(v));

        if (std::abs(denom) < 1e-6f)
            return std::nullopt; // Cannot project (ray is parallel to image plane)

        float lambda = -QVector3D::dotProduct(QVector3D(viewDirection),
                                              QVector3D(centerOfProjection - principalPoint)) / denom;
        projected[i] = centerOfProjection + lambda * v;
    }

    return projected;
}

//
// ========== addCube() ==========
// Registers a cube for projection and stores the result if valid
//
void PerspectiveCamera::addCube(const Cube& cube)
{
    auto cubePoints = cube.getPoints();
    auto projection = projectCube(cubePoints);

    if (projection.has_value())
        projectedObjects.push_back(projection.value());
}

//
// ========== getProjectedCubes() ==========
// Returns the cached projections
//
const std::vector<std::array<QVector4D, 8>>& PerspectiveCamera::getProjectedCubes() const {
    return projectedObjects;
}

//
// ========== triangulatePoint() ==========
// Reconstructs a 3D point from its 2D projections in two camera views
//
/*
QVector4D PerspectiveCamera::triangulatePoint(const QVector4D& p1,
                                              const PerspectiveCamera& cam2,
                                              const QVector4D& p2) const
{
    QVector3D c1 = QVector3D(centerOfProjection);
    QVector3D c2 = QVector3D(cam2.centerOfProjection);
    QVector3D d1 = QVector3D(p1) - c1;
    QVector3D d2 = QVector3D(p2) - c2;

    d1.normalize(); d2.normalize();

    QVector3D n = QVector3D::crossProduct(d1, d2).normalized();
    float t = QVector3D::dotProduct((c2 - c1), QVector3D::crossProduct(n, d2)) /
              QVector3D::dotProduct(d1, QVector3D::crossProduct(n, d2));

    QVector3D p = c1 + t * d1;
    return to4D(p);
}
*/
//
// ========== triangulate() (static) ==========
// Static variant used outside the calling object
//
QVector4D PerspectiveCamera::triangulate(const PerspectiveCamera& cam1,
                                         const QVector4D& p1,
                                         const PerspectiveCamera& cam2,
                                         const QVector4D& p2)
{
    QVector3D c1 = QVector3D(cam1.centerOfProjection);
    QVector3D c2 = QVector3D(cam2.centerOfProjection);
    QVector3D d1 = QVector3D(p1) - c1;
    QVector3D d2 = QVector3D(p2) - c2;

    d1.normalize(); d2.normalize();

    QVector3D n = QVector3D::crossProduct(d1, d2).normalized();
    float t = QVector3D::dotProduct((c2 - c1), QVector3D::crossProduct(n, d2)) /
              QVector3D::dotProduct(d1, QVector3D::crossProduct(n, d2));

    QVector3D p = c1 + t * d1;
    return to4D(p);
}

//
// ========== recomputeViewDirections() ==========
// Rebuilds viewDirection, upVector, and rightVector from the cameraToWorld matrix
//
void PerspectiveCamera::recomputeViewDirections()
{
    QMatrix4x4 R = cameraToWorld;

    rightVector = QVector3D(R.column(0)).normalized();
    upVector = QVector3D(R.column(1)).normalized();
    viewDirection = -QVector3D(R.column(2)).normalized();  // negative Z
}

//
// ========== clearProjectedCubes() ==========
// Empties all previous projections (e.g., after transformation)
//
void PerspectiveCamera::clearProjectedCubes() {
    projectedObjects.clear();
}
