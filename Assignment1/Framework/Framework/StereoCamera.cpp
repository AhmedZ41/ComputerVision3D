#include "StereoCamera.h"
#include "GLConvenience.h"
#include "QtConvenience.h"
#include "SceneManager.h"


// === Constructor ===
// Stores pointers to the left and right perspective cameras
StereoCamera::StereoCamera(PerspectiveCamera* left, PerspectiveCamera* right)
    : camLeft(left), camRight(right)
{
    type = SceneObjectType::ST_STEREO_CAMERA;
}

// === Draw method ===
// Renders both cameras and all reconstructed cubes
void StereoCamera::draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const {
    camLeft->draw(renderer, color, lineWidth);
    camRight->draw(renderer, color, lineWidth);

    for (auto cube : reconstructedCubes) {
        cube->draw(renderer, QColor(0, 0, 255), lineWidth); // Reconstructed cubes in blue
    }
}

// === Apply transformation to the stereo setup ===
// Applies an affine transform to both cameras and the reconstructed geometry
void StereoCamera::affineMap(const QMatrix4x4& matrix) {
    camLeft->affineMap(matrix);
    camRight->affineMap(matrix);
    for (auto cube : reconstructedCubes) {
        cube->affineMap(matrix);
    }
}


// === Stereo reconstruction using triangulation (Part 2) ===
void StereoCamera::reconstructFromStereo() {
    reconstructedCubes.clear();


    // === [Legacy] Dummy reconstruction for testing ===
    /*
    for (const auto& cube : camLeft->getProjectedCubes()) {
        Cube* dummyCube = new Cube(cube[0], 0.2f); // Just place a small cube at one corner
        reconstructedCubes.push_back(dummyCube);
    }
    */

    const auto& projList1 = camLeft->getProjectedObjects();
    const auto& projList2 = camRight->getProjectedObjects();

    for (size_t i = 0; i < projList1.size(); ++i) {
        const auto& proj1 = projList1[i];
        const auto& proj2 = projList2[i];

        std::array<QVector4D, 8> reconstructed;

        // Triangulate each vertex of the cube
        for (int j = 0; j < 8; ++j) {
            reconstructed[j] = PerspectiveCamera::triangulate(*camLeft, proj1[j], *camRight, proj2[j]);
        }

        // Construct the 3D cube from triangulated vertices
        Cube* reconstructedCube = new Cube(reconstructed);

        // Translate reconstructed cube to avoid overlap with originals

        QMatrix4x4 shift;
        shift.translate(0.0f, 0.0f, -3.0f);
        reconstructedCube->affineMap(shift);


        reconstructedCubes.push_back(reconstructedCube);
    }
}


void StereoCamera::reconstructFromStereo(float errorInDegrees) {
    reconstructedCubes.clear();

    // Apply rotation error to right camera
    if (errorInDegrees != 0.0f) {
        QMatrix4x4 rotation;
        rotation.setToIdentity();
        rotation.rotate(errorInDegrees, QVector3D(0, 1, 0)); // Y-axis rotation
        camRight->affineMap(rotation);
        camRight->recomputeViewDirections();  // Ensure viewDirection is updated

    }
    //left cam
    /*
    if (errorInDegrees != 0.0f) {
        QMatrix4x4 rotation;
        rotation.setToIdentity();
        rotation.rotate(errorInDegrees, QVector3D(0, 1, 0)); // Y-axis rotation
        camLeft->affineMap(rotation);
        camLeft->recomputeViewDirections();  // Ensure viewDirection is updated

    }
*/
    const auto& projList1 = camLeft->getProjectedObjects();
    const auto& projList2 = camRight->getProjectedObjects();

    for (size_t i = 0; i < projList1.size(); ++i) {
        const auto& proj1 = projList1[i];
        const auto& proj2 = projList2[i];

        std::array<QVector4D, 8> reconstructed;

        for (int j = 0; j < 8; ++j) {
            reconstructed[j] = PerspectiveCamera::triangulate(*camLeft, proj1[j], *camRight, proj2[j]);
        }

        Cube* reconstructedCube = new Cube(reconstructed);
/*
        QMatrix4x4 shift;
        shift.translate(0.0f, 0.0f, -3.0f); // Z offset for visibility
        reconstructedCube->affineMap(shift);
*/
        reconstructedCubes.push_back(reconstructedCube);
    }

}
