//
// (c) Nico Brügel, 2021
// (c) Georg Umlauf, 2021+2022+2024
//
#include "glwidget.h"
#include <QtGui>

#if defined(__APPLE__)
// we're on macOS and according to their documentation Apple hates developers
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#elif defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
// windows, even if it's case insensitive QT-Create might generate a warning
#include <gl/GL.h>
#include <gl/GLU.h>
#else
// hopefully on linux
// If can't be found, ensure that the following is installed:
// libglu1-mesa-dev and/or mesa-common-dev
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <QApplication>
#include <QtGui>
#include <QMouseEvent>
#include <QFileDialog>
#include <QMessageBox>

#include <cassert>
#include <iostream>

#include "Axes.h"
#include "Plane.h"
#include "PointCloud.h"

using namespace std;
using namespace Qt;

GLWidget::GLWidget(QWidget* parent) : QOpenGLWidget(parent), pointSize(5)
{
    // enable mouse-events, even if no mouse-button is pressed -> yields smoother mouse-move reactions
    setMouseTracking(true);

    // setup render camera and connect its signals
    renderer = new RenderCamera();
    renderer->reset();
    connect(renderer, &RenderCamera::changed, this, &GLWidget::onRendererChanged);

    // setup the scene
    sceneManager.push_back(new Axes(E0,QMatrix4x4()));    // the global world coordinate system
    sceneManager.push_back(new Plane(E0+4*E3,-E3));       // some plane

    // TODO: Assignment 1, Part 1
    //       Add here your own new 3d scene objects, e.g. cubes, hexahedra, etc.,
    //       analog to line 50 above and the respective Axes-class
    //

    // TODO: Assignement 1, Part 2
    //       Add here your own new scene object that represents a perspective camera.
    //       Its draw-method should draw all relevant camera parameters, e.g. image plane, view axes, etc.
    //

    // TODO: Assignement 1, Part 3
    //       Add to your perspective camera methods to project the other scene objects onto its image plane
    //       and to draw the projected objects. These methods have to be invoked in Scene.cpp/Scene::draw.
    //

    // TODO: Assignement 2, Part 1 - 3
    //       Add here your own new scene object that represents a stereo camera pair.
    //       - Part 1: Its draw-method should draw all relevant parameters of both cameras, e.g. image planes, view axes, etc.
    //       - Part 1: Its projection method should project the other scene objects onto their image planes
    //         and draw the projected objects.
    //       - Part 2: Its reconstruction method should reconstruct the 3d geometry of the other scene objects from their stereo projections.
    //       - Part 3: Its reconstruction method should reconstruct the 3d geometry of the other scene objects from misaligned stereo projections.
    //       - This has to be used in Scene.cpp/Scene::draw.
    //
}

//
//  destructor has nothing to do, since it's under Qt control
//
GLWidget::~GLWidget()
{
}

//
//  initializes the canvas and OpenGL context
//
void GLWidget::initializeGL()
{
      // ensure GL flags
      glEnable(GL_POINT_SMOOTH);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glClearColor(0.4f,0.4f,0.4f,1);                       // screen background color
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);               //required for gl_PointSize
}

//
//  redraws the canvas
//
void GLWidget::paintGL()
{
    renderer->setup();

    sceneManager.draw(*renderer,COLOR_SCENE);
}

//
//  reacts on resize events
//
void GLWidget::resizeGL(int width, int height)
{
    QMatrix4x4 projectionMatrix;
    projectionMatrix.setToIdentity();
    projectionMatrix.perspective(70.0f, GLfloat(width) / GLfloat(height), 0.01f, 100.0f);
    renderer->setProjectionMatrix(projectionMatrix);
}

//
//  reacts on mouse-wheel events
//
void GLWidget::wheelEvent(QWheelEvent* event)
{
    // read the wheel angle and move renderer in/out
    if (event->angleDelta().y() > 0) renderer->forward ();
    else                             renderer->backward();
}

//
//  reacts on key-release events
//
void GLWidget::keyReleaseEvent(QKeyEvent * event)
{
    switch ( event->key() )
    {
        // release renderer's axis of rotation
      case Key_X: X_Pressed=false; break;
      case Key_Y: Y_Pressed=false; break;
        // for unhandled events, call keyReleaseEvent of parent class
      default: QWidget::keyReleaseEvent(event); break;
    }
    update();
}

//
//  reacts on key-press events
//
void GLWidget::keyPressEvent(QKeyEvent * event)
{
    switch ( event->key() )
    {
        // trigger translation of renderer using keyboard
    case Key_4:
    case Key_Left:     renderer->left    (); break;
    case Key_6:
    case Key_Right:    renderer->right   (); break;
    case Key_9:
    case Key_PageUp:   renderer->forward (); break;
    case Key_3:
    case Key_PageDown: renderer->backward(); break;
    case Key_8:
    case Key_Up:       renderer->up      (); break;
    case Key_2:
    case Key_Down:     renderer->down    (); break;
        // reset renderer's position
    case Key_R:        renderer->reset   (); break;
        // clamp renderer's axis of rotation
    case Key_X:        X_Pressed=true;       break;
    case Key_Y:        Y_Pressed=true;       break;    // translate point cloud
    case Key_Z: {
        QMatrix4x4 A;
        A.translate(0.0f,0.0f,event->modifiers()&ShiftModifier?-0.1f:0.1f);
        for (auto s: sceneManager) if (s->getType()==SceneObjectType::ST_POINT_CLOUD) s->affineMap(A);
        break;
    }
        // quit application
    case Key_Q:
    case Key_Escape: QApplication::instance()->quit(); break;
        // for unhandled events call keyPressEvent of parent class
    default: QWidget::keyPressEvent(event);  break;
    }
    update();
}

//
//  reacts on mouse-move events
//
void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    QPoint d = event->pos()-prevMousePosition;
    prevMousePosition = event->pos();

    // if left-mouse-button is pressed, trigger rotation of renderer
    if (event->buttons() & Qt::LeftButton)
    {
        renderer->rotate(X_Pressed?0:d.y(), Y_Pressed?0:d.x(), 0);
    }
    // if right-mouse-button is pressed, trigger translation of renderer
    else if ( event->buttons() & Qt::RightButton)
    {
        if (d.x() < 0) renderer->right();
        if (d.x() > 0) renderer->left();
        if (d.y() < 0) renderer->down();
        if (d.y() > 0) renderer->up();
    }
}

//
//  triggers re-draw, if renderer emits changed-signal
//
void GLWidget::onRendererChanged()
{
    update();
}

//
// updates the point size in each point cloud in the scene management
//
void GLWidget::setPointSize(int size)
{
    assert(size > 0);
    pointSize = size;
    for (auto s: sceneManager) if (s->getType()==SceneObjectType::ST_POINT_CLOUD) reinterpret_cast<PointCloud*>(s)->setPointSize(unsigned(pointSize));
    update();
}

//
// 1. reacts on push button click
// 2. opens file dialog
// 3. loads ply-file data to new point cloud
// 4. attaches new point cloud to scene management
//
void GLWidget::openFileDialog()
{
    const QString filePath   = QFileDialog::getOpenFileName(this, tr("Open PLY file"), "../data", tr("PLY Files (*.ply)"));
    PointCloud*   pointCloud = new PointCloud;

    if (!filePath.isEmpty() && pointCloud)
    {
         cout << filePath.toStdString() << endl;
         pointCloud->loadPLY(filePath);
         pointCloud->setPointSize(unsigned(pointSize));
         sceneManager.push_back(pointCloud);
         update();
         return;
     }
     delete pointCloud;
}

//
// controls radio button clicks
//
void GLWidget::radioButtonClicked()
{
    // TODO: toggle to
    QMessageBox::warning(this, "Feature" ,"Some things are missing here. Implement yourself, if necessary.");
    if (sender()) {
       QString      name = sender()->objectName();
       if (name=="radioButton_1") {};
       if (name=="radioButton_2") {};
       update();
    }
}

//
// controls check box clicks
//
void GLWidget::checkBoxClicked()
{
    QMessageBox::warning(this, "Feature" ,"ups hier fehlt noch was");
}

//
// controls spin box changes
//
void GLWidget::spinBoxValueChanged(int)
{
    QMessageBox::warning(this, "Feature" ,"ups hier fehlt noch was");
}

