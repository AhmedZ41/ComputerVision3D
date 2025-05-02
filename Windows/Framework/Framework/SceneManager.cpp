//
//  A very simple class for rudimentary scene management
//
//  (c) Georg Umlauf, 2021+2022
//

#include "SceneManager.h"
#include "PerspectiveCamera.h"
#include "StereoCamera.h"

using enum SceneObjectType;
//
// iterates all objects under its control and has them drawn by the renderer
//
void SceneManager::draw(const RenderCamera& renderer, const QColor& color) const
{
    for (auto obj : *this) if (obj) {
        switch (obj->getType()) {
        case ST_AXES:
            obj->draw(renderer,COLOR_AXES,2.0f);
            break;
        case ST_PLANE:
            obj->draw(renderer,COLOR_PLANE,0.3f);
            break;
        case ST_CUBE:
        case ST_HEXAHEDRON:
            obj->draw(renderer,color,2.0f);
            break;
        case ST_POINT_CLOUD:
            obj->draw(renderer,COLOR_POINT_CLOUD,3.0f);     // last argument unused
            break;
        case ST_PERSPECTIVE_CAMERA: {
            // TODO: Assignement 1, Part 3
            // This is the place to invoke the perspective camera's projection method and draw the projected objects.
            obj->draw(renderer, COLOR_CAMERA, 2.0f);

            auto* cam = dynamic_cast<const PerspectiveCamera*>(obj);
            if (cam) {
                cam->drawProjections(renderer, *this, Qt::magenta);
            }
            break;
        }
        case ST_STEREO_CAMERA:
            // TODO: Assignement 2, Part 1 - 3
            // Part 1: This is the place to invoke the stereo camera's projection method and draw the projected objects.
            // Part 2: This is the place to invoke the stereo camera's reconstruction method.
            // Part 3: This is the place to invoke the stereo camera's reconstruction method using misaligned stereo cameras.
            auto* stereo = static_cast<StereoCamera*>(obj);
            stereo->reconstruct(*this);
            stereo->draw(renderer, Qt::magenta, 2.0f);
            stereo->drawProjections(renderer, *this);

            break;
        }
    }
}
