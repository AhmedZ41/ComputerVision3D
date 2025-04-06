#include "PerspectiveCamera.h"
#include "GLConvenience.h"
#include "QtConvenience.h"

#include <cmath>

// Constructor
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
    // Set object type
    type = SceneObjectType::ST_PERSPECTIVE_CAMERA;

    // Compute camera coordinate system and camera-to-world matrix
    computeCameraCoordinateSystem();

    // Compute the principal point H = N + f * (-viewDirection)
    QVector3D imagePlaneCenter = QVector3D(centerOfProjection) + (-viewDirection) * focalLength;
    principalPoint = to4D(imagePlaneCenter);

    // Create Axes (local coordinate system at the camera center)
    localAxes = new Axes(centerOfProjection, cameraToWorld);

    // Compute 4 corners of image plane in camera coordinates
    QVector3D x = rightVector * (imagePlaneWidth / 2.0f);
    QVector3D y = upVector    * (imagePlaneHeight / 2.0f);

    QVector3D tl = imagePlaneCenter - x + y; // top left
    QVector3D tr = imagePlaneCenter + x + y; // top right
    QVector3D br = imagePlaneCenter + x - y; // bottom right
    QVector3D bl = imagePlaneCenter - x - y; // bottom left

    // Create the image plane
    imagePlane = new Plane(to4D(tl), to4D(viewDirection)); // normal is viewDirection
    // We override the draw method, so actual quad is drawn in draw()
}

// Destructor
PerspectiveCamera::~PerspectiveCamera()
{
    delete localAxes;
    delete imagePlane;
}

// Draw method
void PerspectiveCamera::draw(const RenderCamera& renderer,
                             const QColor& color,
                             float lineWidth) const
{
    // 1. Draw the camera axes at center
    localAxes->draw(renderer, color, lineWidth);

    // 2. Draw the center of projection N
    renderer.renderPoint(centerOfProjection, color, 5.0f);

    // 3. Draw the principal point H on the image plane
    renderer.renderPoint(principalPoint, QColor(255, 0, 255), 5.0f); // magenta

    // 4. Draw a line from N to H (camera viewing direction)
    renderer.renderLine(centerOfProjection, principalPoint, QColor(255, 0, 0), 2.0f);

    // 5. Draw the image plane (4 corners computed in constructor)
    QVector3D x = rightVector * (imagePlaneWidth / 2.0f);
    QVector3D y = upVector    * (imagePlaneHeight / 2.0f);
    QVector3D center = QVector3D(principalPoint);

    QVector3D tl = center - x + y;
    QVector3D tr = center + x + y;
    QVector3D br = center + x - y;
    QVector3D bl = center - x - y;

    renderer.renderPlane(tl, tr, br, bl, QColor(255, 0, 0), 0.3f); // transparent red
}

// Apply an affine transformation to the camera
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

    // Update local axes
    localAxes->affineMap(matrix);
}

// Compute right vector and build cameraToWorld matrix
void PerspectiveCamera::computeCameraCoordinateSystem()
{
    // Compute orthonormal basis
    rightVector = QVector3D::crossProduct(upVector, viewDirection).normalized(); // x_cam
    upVector    = QVector3D::crossProduct(viewDirection, rightVector).normalized(); // y_cam


    // Create rotation matrix from camera basis
    QMatrix4x4 R;
    R.setColumn(0, to4D(rightVector));     // x_cam
    R.setColumn(1, to4D(upVector));        // y_cam
    R.setColumn(2, to4D(-viewDirection));  // z_cam (looking into -z)
    R.setColumn(3, QVector4D(0, 0, 0, 1)); // Homogeneous identity

    // Translation to camera position
    QMatrix4x4 T;
    T.setToIdentity();
    T.translate(QVector3D(centerOfProjection));

    // Full camera-to-world matrix
    cameraToWorld = T * R;
}
