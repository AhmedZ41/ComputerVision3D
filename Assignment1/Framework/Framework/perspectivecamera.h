#pragma once

#include "SceneObject.h"
#include "Axes.h"
#include "Plane.h"
#include "SceneManager.h"

class Cube;  // 👈 Forward declaration


class PerspectiveCamera : public SceneObject {
private:
    // ===== Camera Parameters =====
    QVector4D centerOfProjection;    // N: camera center (eye)
    QVector3D viewDirection;         // View direction (usually negative z-axis)
    QVector3D upVector;              // Up direction (typically y-axis)
    QVector3D rightVector;           // Right direction (x-axis), computed via cross product

    float focalLength;              // Distance from center to image plane
    float imagePlaneWidth;          // Width of image plane (used to size the quad)
    float imagePlaneHeight;         // Height of image plane

    QVector4D principalPoint;       // H: image center on image plane

    // ===== Visualization Objects =====
    QMatrix4x4 cameraToWorld;       // Transformation matrix from camera space to world space
    Axes* localAxes;                // Coordinate axes (for visualizing camera orientation)
    Plane* imagePlane;              // A quad representing the image plane
    std::vector<std::array<QVector4D, 8>> projectedObjects; // Stores projected cubes

    // Projection helper
    std::optional<std::array<QVector4D, 8>> projectCube(const std::array<QVector4D, 8>& cubePoints);


public:
    // ===== Constructor =====
    PerspectiveCamera(const QVector4D& position,
                      const QVector3D& lookDir,
                      const QVector3D& up,
                      float focalLength = 1.0f,
                      float width = 1.0f,
                      float height = 0.75f);  // default 4:3 image plane

    virtual ~PerspectiveCamera() override;

    // ===== Transformation & Drawing =====
    virtual void affineMap(const QMatrix4x4& matrix) override;
    virtual void draw(const RenderCamera& renderer,
                      const QColor& color = COLOR_CAMERA,
                      float lineWidth = 2.0f) const override;

    // ===== Getter for Principal Point (used later in projection) =====
    QVector4D getPrincipalPoint() const { return principalPoint; }

    // TODO (Part 3): Method to project 3D points to image plane
    void addCube(const Cube& cube); // Register a cube for projection

    const std::vector<std::array<QVector4D, 8>>& getProjectedCubes() const;
    QVector4D triangulatePoint(const QVector4D& p1, const PerspectiveCamera& cam2, const QVector4D& p2) const;

    const std::vector<std::array<QVector4D, 8>>& getProjectedObjects() const {
        return projectedObjects;
    }

    static QVector4D triangulate(const PerspectiveCamera& cam1,
                                 const QVector4D& p1,
                                 const PerspectiveCamera& cam2,
                                 const QVector4D& p2);
    void recomputeViewDirections();
    void clearProjectedCubes();
    SceneManager* sceneReference = nullptr;




private:
    // Helper to update cameraToWorld and derived members
    void computeCameraCoordinateSystem();


};
