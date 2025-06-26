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
#include "Cube.h"
#include "PerspectiveCamera.h"
#include "StereoCamera.h"
#include "KDNode.h"
#include "PointSet.h"
#include "KDTree.h"
#include "OctreeNode.h"
#include "PCA.h"
#include <Eigen/Dense>
#include "PCAAxes.h"







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
    //sceneManager.push_back(new Axes(E0,QMatrix4x4()));    // the global world coordinate system
    // sceneManager.push_back(new Plane(E0+4*E3,-E3));       // some plane

    // TODO: Assignment 1, Part 1
    //       Add here your own new 3d scene objects, e.g. cubes, hexahedra, etc.,
    //       analog to line 50 above and the respective Axes-class
    //
    //sceneManager.push_back(new Cube(E0 + 6*E3 + E1 + E2, .5f));
    //sceneManager.push_back(new Cube(E0 + 9*E3 + -1*E1 + 1*E2, .5f));
    //sceneManager.push_back(new Cube(E0 + 5*E3 + -1*E1 + -0.5*E2, .5f));
    //sceneManager.push_back(new Cube(E0 + 5*E3 + -5*E1 + -0.5*E2, .5f));




    // TODO: Assignement 1, Part 2
    //       Add here your own new scene object that represents a perspective camera.
    //       Its draw-method should draw all relevant camera parameters, e.g. image plane, view axes, etc

    //
    /*
    auto cam = new PerspectiveCamera(E0 + 2*E1+ 1*E3,
                                     QVector3D(0, 0, -1),
                                     QVector3D(0, 1, 0),
                                     2.0f,
                                     1.5f, 1.5f);

    sceneManager.push_back(cam);
    */

    // TODO: Assignement 1, Part 3
    //       Add to your perspective camera methods to project the other scene objects onto its image plane
    //       and to draw the projected objects. These methods have to be invoked in Scene.cpp/Scene::draw.
    //

/*
    // Add all existing cubes to camera for projection
    for (auto s : sceneManager) {
        if (s->getType() == SceneObjectType::ST_CUBE) {
            cam->addCube(*reinterpret_cast<Cube*>(s));
        }
    }
*/

    // TODO: Assignement 2, Part 1 - 3
    //       Add here your own new scene object that represents a stereo camera pair.
    //       - Part 1: Its draw-method should draw all relevant parameters of both cameras, e.g. image planes, view axes, etc.
    //       - Part 1: Its projection method should project the other scene objects onto their image planes
    //         and draw the projected objects.
    //       - Part 2: Its reconstruction method should reconstruct the 3d geometry of the other scene objects from their stereo projections.
    //       - Part 3: Its reconstruction method should reconstruct the 3d geometry of the other scene objects from misaligned stereo projections.
    //       - This has to be used in Scene.cpp/Scene::draw.
    //

    // === Add second perspective camera ===
    /*
    auto cam2 = new PerspectiveCamera(
        E0 - 1*E1 + 1*E3,                // position
        QVector3D(0, 0, -1),             // view direction
        QVector3D(0, 1, 0),              // up vector
        2.0f, 1.5f, 1.5f                 // focal length, image plane size
        );
    sceneManager.push_back(cam2);
*/


    // === Simulate camera misalignment (Part 3) ===
/*
    QMatrix4x4 rotation;
    rotation.setToIdentity();
    rotation.rotate(0.0f, QVector3D(0, 0, 0)); // 15° rotation around Y-axis
    cam2->affineMap(rotation);
    cam2->recomputeViewDirections(); // Update view direction after transform




    // === Project all cubes onto cam2's image plane ===
    for (auto s : sceneManager) {
        if (s->getType() == SceneObjectType::ST_CUBE)
            cam2->addCube(*reinterpret_cast<Cube*>(s));
    }

*/


    // === Manual triangulation & display (used for debugging) ===
    /*
    const auto& projList1 = cam->getProjectedObjects();
    const auto& projList2 = cam2->getProjectedObjects();

    for (size_t i = 0; i < projList1.size(); ++i) {
        const auto& proj1 = projList1[i];
        const auto& proj2 = projList2[i];

        std::array<QVector4D, 8> reconstructed;
        for (int j = 0; j < 8; ++j) {
            reconstructed[j] = cam->triangulate(*cam, proj1[j], *cam2, proj2[j]);
        }

        Cube* reconstructedCube = new Cube(reconstructed);
        sceneManager.push_back(reconstructedCube);
    }
    */



    // === Use StereoCamera class to handle projection and reconstruction ===
    /*
    auto stereo = new StereoCamera(cam, cam2);
    stereo->reconstructFromStereo();
    sceneManager.push_back(stereo);
*/
/*
    auto stereo = new StereoCamera(cam, cam2);
    stereo->reconstructFromStereo(0.0f); // 1° error
    sceneManager.push_back(stereo);

    // === Project all cubes onto cam2's image plane ===
    for (auto s : sceneManager) {
        if (s->getType() == SceneObjectType::ST_CUBE)
            cam2->addCube(*reinterpret_cast<Cube*>(s));
    }
            */

    // ==== Load bunny and encapsulated KDTree into scene ====
    auto* bunny = new PointCloud();
    QString bunnyPath = "/Users/ahmedadnan/Desktop/HTWG/S6/Computervision-3D/ComputerVision3D/Assignment1/Framework/data/bunny.ply";
    QFileInfo info(bunnyPath);
    if (!info.exists()) {
        qDebug() << "File does not exist:" << bunnyPath;
    } else {
        qDebug() << "Found bunny.ply at:" << info.absoluteFilePath();
    }

    if (bunny->loadPLY("/Users/ahmedadnan/Desktop/HTWG/S6/Computervision-3D/ComputerVision3D/Assignment1/Framework/data/bunny.ply")) {
        bunny->setPointSize(unsigned(pointSize));
        sceneManager.push_back(bunny);

        // Encapsulated KDTree scene object
        auto* tree = new KDTree(bunny);
        sceneManager.push_back(tree);
    }



    // === Load bunny and build Octree ===
    auto* bunnyOct = new PointCloud();
    
    if (bunnyOct->loadPLY("/Users/ahmedadnan/Desktop/HTWG/S6/Computervision-3D/ComputerVision3D/Assignment1/Framework/data/bunny.ply")) {
        bunnyOct->setPointSize(unsigned(pointSize));
        sceneManager.push_back(bunnyOct);

        // === Compute and print centroid ===
        QVector4D centroid = PCA::computeCentroid(*bunnyOct);
        qDebug() << "Centroid of bunny:" << centroid;

        Eigen::Matrix3f cov = PCA::computeCovarianceMatrix(*bunnyOct, centroid);
        qDebug() << "Covariance matrix:\n"
                 << cov(0,0) << cov(0,1) << cov(0,2) << "\n"
                 << cov(1,0) << cov(1,1) << cov(1,2) << "\n"
                 << cov(2,0) << cov(2,1) << cov(2,2);


        auto [eigenvectors, eigenvalues] = PCA::computeEigenvectorsAndValues(cov);
        // Add visual representation of PCA axes as a SceneObject
        auto* pcaAxes = new PCAAxes(centroid, eigenvectors);
        sceneManager.push_back(pcaAxes);


        qDebug() << "Eigenvalues:";
        for (int i = 0; i < 3; ++i)
            qDebug() << "λ[" << i << "] =" << eigenvalues(i);

        qDebug() << "Eigenvectors (columns):";
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3f v = eigenvectors.col(i);
            qDebug() << "v[" << i << "] =" << v(0) << v(1) << v(2);
        }

        // ==========================================
        // Part 2.1: Create Transformed Bunny
        // ==========================================
        
        // Create a copy of the original bunny
        auto* transformedBunny = new PointCloud(*bunnyOct);  // Copy constructor
        
        // Apply a known transformation (rotation + translation)
        QMatrix4x4 knownTransform;
        knownTransform.setToIdentity();
        knownTransform.rotate(20.0f, QVector3D(1, 1, 1));    // 30° rotation around Y-axis
        knownTransform.translate(1.5f, 0.0f, 0.0f);          // Translation in X, Y, Z
        
        // Apply the transformation
        transformedBunny->affineMap(knownTransform);
        
        // Add to scene manager (this will be drawn in a different color via SceneManager)
        sceneManager.push_back(transformedBunny);
        
        qDebug() << "Created transformed bunny with rotation 30° around Y + translation (0.5, 0.2, 0.1)";

// ==========================================
        // Part 2.2: Compute PCA on Transformed Bunny
        // ==========================================
        
        // Compute centroid of transformed bunny
        QVector4D transformedCentroid = PCA::computeCentroid(*transformedBunny);
        qDebug() << "Centroid of transformed bunny:" << transformedCentroid;
        
        // Compute covariance matrix of transformed bunny
        Eigen::Matrix3f transformedCov = PCA::computeCovarianceMatrix(*transformedBunny, transformedCentroid);
        qDebug() << "Transformed covariance matrix:\n"
                 << transformedCov(0,0) << transformedCov(0,1) << transformedCov(0,2) << "\n"
                 << transformedCov(1,0) << transformedCov(1,1) << transformedCov(1,2) << "\n"
                 << transformedCov(2,0) << transformedCov(2,1) << transformedCov(2,2);

        // Compute eigenvectors and eigenvalues of transformed bunny
        auto [transformedEigenvectors, transformedEigenvalues] = PCA::computeEigenvectorsAndValues(transformedCov);
        
        // Add visual representation of transformed PCA axes
        auto* transformedPCAAxes = new PCAAxes(transformedCentroid, transformedEigenvectors);
        sceneManager.push_back(transformedPCAAxes);

        qDebug() << "Transformed Eigenvalues:";
        for (int i = 0; i < 3; ++i)
            qDebug() << "λ_transformed[" << i << "] =" << transformedEigenvalues(i);

        qDebug() << "Transformed Eigenvectors (columns):";
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3f v = transformedEigenvectors.col(i);
            qDebug() << "v_transformed[" << i << "] =" << v(0) << v(1) << v(2);
        }

        // Store variables for next steps
        qDebug() << "\n=== PCA Alignment Data ===";
        qDebug() << "Original centroid:" << centroid;
        qDebug() << "Transformed centroid:" << transformedCentroid;

                // ==========================================
        // Part 2.3: Compute Rotation Matrix for Alignment
        // ==========================================
        
        // Compute rotation matrix R_align = V_orig * V_transformed^T
        Eigen::Matrix3f R_align = eigenvectors * transformedEigenvectors.transpose();
        
        qDebug() << "\n=== Alignment Rotation Matrix ===";
        qDebug() << "R_align = V_orig * V_transformed^T";
        qDebug() << "R_align:\n"
                 << R_align(0,0) << R_align(0,1) << R_align(0,2) << "\n"
                 << R_align(1,0) << R_align(1,1) << R_align(1,2) << "\n"
                 << R_align(2,0) << R_align(2,1) << R_align(2,2);
        
        // Verify it's a proper rotation matrix (optional check)
        Eigen::Matrix3f shouldBeIdentity = R_align * R_align.transpose();
        float determinant = R_align.determinant();
        qDebug() << "Determinant of R_align:" << determinant << "(should be ~1.0)";
        qDebug() << "R * R^T (should be identity):\n"
                 << shouldBeIdentity(0,0) << shouldBeIdentity(0,1) << shouldBeIdentity(0,2) << "\n"
                 << shouldBeIdentity(1,0) << shouldBeIdentity(1,1) << shouldBeIdentity(1,2) << "\n"
                 << shouldBeIdentity(2,0) << shouldBeIdentity(2,1) << shouldBeIdentity(2,2);

        // ==========================================
        // Part 2.4: Compute Translation Vector
        // ==========================================
        
        // Convert centroids to Eigen vectors for computation
        Eigen::Vector3f centroid_orig(centroid.x(), centroid.y(), centroid.z());
        Eigen::Vector3f centroid_transformed(transformedCentroid.x(), transformedCentroid.y(), transformedCentroid.z());
        
        // Compute translation: T_align = centroid_original - R_align * centroid_transformed
        Eigen::Vector3f T_align = centroid_orig - R_align * centroid_transformed;
        
        qDebug() << "\n=== Alignment Translation Vector ===";
        qDebug() << "T_align = centroid_orig - R_align * centroid_transformed";
        qDebug() << "T_align:" << T_align(0) << T_align(1) << T_align(2);


        // ==========================================
        // Part 2.5: Apply Inverse Transformation
        // ==========================================
        
        // Build the 4x4 alignment transformation matrix
        QMatrix4x4 alignmentMatrix;
        alignmentMatrix.setToIdentity();
        
        // Set the rotation part (top-left 3x3)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                alignmentMatrix(i, j) = R_align(i, j);
            }
        }
        
        // Set the translation part (top-right 3x1)
        alignmentMatrix(0, 3) = T_align(0);
        alignmentMatrix(1, 3) = T_align(1);
        alignmentMatrix(2, 3) = T_align(2);
        
        qDebug() << "\n=== 4x4 Alignment Matrix ===";
        qDebug() << "Alignment Matrix:";
        for (int i = 0; i < 4; ++i) {
            qDebug() << alignmentMatrix(i,0) << alignmentMatrix(i,1) << alignmentMatrix(i,2) << alignmentMatrix(i,3);
        }
        
        // Create a copy of the transformed bunny for alignment
        auto* alignedBunny = new PointCloud(*transformedBunny);
        
        // Apply the alignment transformation to bring it back to original position
        alignedBunny->affineMap(alignmentMatrix);
        
        // Add to scene manager for visualization
        sceneManager.push_back(alignedBunny);
        
        qDebug() << "\n=== Alignment Complete ===";
        qDebug() << "Created aligned bunny - should now match the original bunny position";
        qDebug() << "You should see 3 overlapping point clouds:";
        qDebug() << "1. Original bunny (white)";
        qDebug() << "2. Transformed bunny (white, displaced)";
        qDebug() << "3. Aligned bunny (white, should overlap with original)";

        // Compute centroid of aligned bunny to verify alignment
        QVector4D alignedCentroid = PCA::computeCentroid(*alignedBunny);
        qDebug() << "Original centroid:" << centroid;
        qDebug() << "Aligned centroid:" << alignedCentroid;
        qDebug() << "Difference:" << (centroid - alignedCentroid);
  




        // Compute actual bounding box from scaled points
        QVector3D minCorner(std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max());

        QVector3D maxCorner = -minCorner;

        for (const auto& p : *bunnyOct) {
            minCorner.setX(std::min(minCorner.x(), p.x()));
            minCorner.setY(std::min(minCorner.y(), p.y()));
            minCorner.setZ(std::min(minCorner.z(), p.z()));

            maxCorner.setX(std::max(maxCorner.x(), p.x()));
            maxCorner.setY(std::max(maxCorner.y(), p.y()));
            maxCorner.setZ(std::max(maxCorner.z(), p.z()));
        }

        // Convert to QVector4D (homogeneous) format
        QVector4D min(minCorner, 1.0f);
        QVector4D max(maxCorner, 1.0f);

        // Create and add Octree
        auto* octree = new OctreeNode(*bunnyOct, min, max);
        sceneManager.push_back(octree);
    }




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

    sceneManager.draw(*renderer, COLOR_SCENE);
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
        // Toggle KD-tree with 'k'
    case Key_K:
        showKDTree = !showKDTree;
        showOctree = false;  // Ensure only one visualization is active
        break;
        // Toggle Octree with single 'o' press
    case Key_O:
        showOctree = !showOctree;
        showKDTree = false;  // Ensure only one visualization is active
        break;
        // Toggle PCA axes with 'p'
    case Key_P:
        showPCAAxes = !showPCAAxes;
        break;
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

/*
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
}*/

void GLWidget::openFileDialog()
{
    const QString filePath = QFileDialog::getOpenFileName(this, tr("Open PLY file"), "../data", tr("PLY Files (*.ply)"));

    if (filePath.isEmpty())
        return;

    // Load the point cloud
    PointCloud* pointCloud = new PointCloud();
    if (!pointCloud->loadPLY(filePath)) {
        delete pointCloud;
        return;
    }

    pointCloud->setPointSize(unsigned(pointSize));
    sceneManager.push_back(pointCloud);

    // ==== Build KD-tree ====
    std::vector<QVector4D> rawPoints;
    for (const auto& p : *pointCloud)
        rawPoints.push_back(p);

    PointSet fullSet(rawPoints);
    KDNode* kdTreeRoot = new KDNode(fullSet);  // Uses default depth=0, maxDepth=3, minPoints=10
    sceneManager.push_back(kdTreeRoot);

    // Trigger re-render
    update();
}


//
// controls radio button clicks
//constr
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
