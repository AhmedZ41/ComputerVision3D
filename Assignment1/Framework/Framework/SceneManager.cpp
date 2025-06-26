//
//  A very simple class for rudimentary scene management
//
//  (c) Georg Umlauf, 2021+2022
//

#include "SceneManager.h"

using enum SceneObjectType;
//
// iterates all objects under its control and has them drawn by the renderer
//
void SceneManager::draw(const RenderCamera& renderer, const QColor& color) const
{
    static int pointCloudCount = 0;  // Static counter to track which point cloud we're drawing
    pointCloudCount = 0;  // Reset counter at start of each draw call
    for (auto obj : *this) if (obj) {
            switch (obj->getType()) {
            case ST_AXES:
                //obj->draw(renderer,COLOR_AXES,2.0f);
                break;
            case ST_PLANE:
                //obj->draw(renderer,COLOR_PLANE,0.3f);
                break;
            case ST_CUBE:
            case ST_HEXAHEDRON:
                obj->draw(renderer,color,2.0f);
                break;
            case ST_POINT_CLOUD:
                {
                    QColor pointCloudColor;
                    switch(pointCloudCount) {
                        case 0: pointCloudColor = QColor(255, 255, 255); break;  // 1st bunny: white (with KDTree)
                        case 1: pointCloudColor = QColor(255, 255, 255); break;  // 2nd bunny: white (original)
                        case 2: pointCloudColor = QColor(255, 255, 0);   break;  // 3rd bunny: yellow (transformed)
                        case 3: pointCloudColor = QColor(255, 255, 0);   break;  // 4th bunny: yellow (aligned)
                        default: pointCloudColor = QColor(255, 255, 255); break; // Default: white
                    }
                    obj->draw(renderer, pointCloudColor, 3.0f);
                    pointCloudCount++;
                }
                break;
            case ST_KD_TREE:
                //obj->draw(renderer, QColor(100, 100, 255), 2.0f);  // draw kd-tree splitting planes
                break;
            case ST_OCTREE:
                //obj->draw(renderer, QColor(255, 165, 0), 1.0f); // Orange color
                break;
            case ST_PCA_AXES:
                obj->draw(renderer, QColor(255, 255, 0), 1.0f);
                break;




            case ST_PERSPECTIVE_CAMERA:
                //obj->draw(renderer, COLOR_CAMERA, 2.0f);
                break;
                // TODO: Assignement 1, Part 3
                // This is the place to invoke the perspective camera's projection method and draw the projected objects.

                break;

            case ST_STEREO_CAMERA:
               // obj->draw(renderer, COLOR_CAMERA, 2.0f); // Part 2 draw call
                break;
                // TODO: Assignement 2, Part 1 - 3
                // Part 1: This is the place to invoke the stereo camera's projection method and draw the projected objects.
                // Part 2: This is the place to invoke the stereo camera's reconstruction method.
                // Part 3: This is the place to invoke the stereo camera's reconstruction method using misaligned stereo cameras.
                break;
            }

        }
}
