#pragma once

#include "SceneObject.h"
#include "PerspectiveCamera.h"
#include "Cube.h"
#include <optional>

class StereoCamera : public SceneObject {
public:
    StereoCamera(PerspectiveCamera* left, PerspectiveCamera* right);

    void draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const override;
    void affineMap(const QMatrix4x4& matrix) override;

    void reconstructFromStereo(); // part 2
    void reconstructFromStereo(float errorInDegrees = 0.0f);
    SceneManager* sceneReference = nullptr;



private:
    PerspectiveCamera* camLeft;
    PerspectiveCamera* camRight;
    std::vector<Cube*> reconstructedCubes;
};
