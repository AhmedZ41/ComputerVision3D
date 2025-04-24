#include "StereoCamera.h"
#include "GLConvenience.h"
#include "QtConvenience.h"

StereoCamera::StereoCamera(PerspectiveCamera* left, PerspectiveCamera* right)
    : camLeft(left), camRight(right)
{
    type = SceneObjectType::ST_STEREO_CAMERA;
}

void StereoCamera::draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const {
    camLeft->draw(renderer, color, lineWidth);
    camRight->draw(renderer, color, lineWidth);

    for (auto cube : reconstructedCubes) {
        cube->draw(renderer, QColor(0, 255, 0), lineWidth); // Draw green reconstructed cube
    }
}

void StereoCamera::affineMap(const QMatrix4x4& matrix) {
    camLeft->affineMap(matrix);
    camRight->affineMap(matrix);
    for (auto cube : reconstructedCubes) {
        cube->affineMap(matrix);
    }
}

// Dummy stereo reconstruction for now
void StereoCamera::reconstructFromStereo() {
    reconstructedCubes.clear();

    // Just re-add the same cubes from left camera for now
    for (const auto& cube : camLeft->getProjectedCubes()) {
        Cube* dummyCube = new Cube(cube[0], 0.2f); // Fake cube at first corner for visualization
        reconstructedCubes.push_back(dummyCube);
    }
}
