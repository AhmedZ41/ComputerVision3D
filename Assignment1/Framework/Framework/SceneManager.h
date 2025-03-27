//
//  A very simple class for rudimentary scene management
//
//  (c) Georg Umlauf, 2021+2022
//
#pragma once

#include "SceneObject.h"
#include "RenderCamera.h"

#include <vector>
#include <QObject>
#include <QColor>

class SceneManager: public QObject, public std::vector<SceneObject*>
{
private:
public:
    SceneManager(QObject* parent=nullptr): QObject(parent) {}
    ~SceneManager () override {}

    // Iterates all objects under its control and has them drawn by the renderer
    //
    // ATTENTION: You have to inherit from SceneObject, i.e., you MUST implement your own
    //            draw- and affineMap-method in your SceneObjects!!!!!
    //            This draw-method is going to use these methods.
    //
    void draw(const RenderCamera& renderer,
              const QColor      & color    = COLOR_SCENE) const;
};

