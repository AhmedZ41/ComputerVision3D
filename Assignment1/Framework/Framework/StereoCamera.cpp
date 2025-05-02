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
        cube->draw(renderer, QColor(0, 0, 255), lineWidth); // Draw reconstructed cube
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
    /*
     * for (const auto& cube : camLeft->getProjectedCubes()) {
        Cube* dummyCube = new Cube(cube[0], 0.2f); // Fake cube at first corner for visualization
        reconstructedCubes.push_back(dummyCube);
    }*/
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

        // Shift it for better display
        QMatrix4x4 shift;
        shift.translate(0.0f, 0.0f, -3.0f); // adjust axis and amount as needed
        reconstructedCube->affineMap(shift);

        reconstructedCubes.push_back(reconstructedCube);
    }


}
