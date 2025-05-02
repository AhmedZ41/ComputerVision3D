//
// This is the camera for the rendering of the scene ONLY.
//
// (c) Georg Umlauf, 2021
//
#include "RenderCamera.h"
#include "GLConvenience.h"
#include "QtConvenience.h"

RenderCamera::RenderCamera(QObject* parent) :
    QObject(parent),
    xRotation(0),
    yRotation(0),
    zRotation(0)
{
    projectionMatrix.setToIdentity();
    cameraMatrix.    setToIdentity();
    worldMatrix.     setToIdentity();
    renderMatrix = getRenderMatrix();
}

void RenderCamera::setup()
{
    // position and angles
    QMatrix4x4 cm;
    cm.setToIdentity();
    cm.translate(position.x(),position.y(),position.z());
    cm.rotate   (xRotation, 1, 0, 0);
    cm.rotate   (yRotation, 0, 1, 0);
    cm.rotate   (zRotation, 0, 0, 1);
    setCameraMatrix(cm);
    // the world is still for now, thus nothing to do for renderCamera->setWorldMatrix
}

void RenderCamera::reset()
{
    xRotation=0;
    yRotation=0;
    zRotation=0;
    setPosition(QVector3D(-0.05f, -0.0f, -0.1f));
    rotate(0, 0, 0);
    renderMatrix = getRenderMatrix();
}

void RenderCamera::forward()
{
    position[2] += TranslationSTEP;
    emit changed();
}


void RenderCamera::backward()
{
    position[2] -= TranslationSTEP;
    emit changed();
}

void RenderCamera::left()
{
    position[0] += TranslationSTEP;
    emit changed();
}

void RenderCamera::right()
{
    position[0] -= TranslationSTEP;
    emit changed();
}

void RenderCamera::up()
{
    position[1] -= TranslationSTEP;
    emit changed();
}

void RenderCamera::down()
{
    position[1] += TranslationSTEP;
    emit changed();
}

void RenderCamera::setPosition(const QVector3D& _position)
{
    position = _position;
}

void RenderCamera::setProjectionMatrix(const QMatrix4x4& P)
{
    projectionMatrix = P;
    renderMatrix = getRenderMatrix();
}

void RenderCamera::setCameraMatrix(const QMatrix4x4& C)
{
    cameraMatrix = C;
    renderMatrix = getRenderMatrix();
}

void RenderCamera::setWorldMatrix(const QMatrix4x4& W)
{
    worldMatrix = W;
    renderMatrix = getRenderMatrix();
}

void RenderCamera::setXRotation(int angle)
{
    angle = angle % (RotationBASE * RotationSTEP);
    if (angle != xRotation) {
        xRotation = angle;
        emit changed();
    }
}

void RenderCamera::setYRotation(int angle)
{
    angle = angle % (RotationBASE * RotationSTEP);
    if (angle != yRotation) {
        yRotation = angle;
        emit changed();
    }
}

void RenderCamera::setZRotation(int angle)
{
    angle = angle % (RotationBASE * RotationSTEP);
    if (angle != zRotation) {
        zRotation = angle;
        emit changed();
    }
}

void RenderCamera::rotate(int dx, int dy, int dz)
{
    setXRotation(xRotation + dx);
    setYRotation(yRotation + dy);
    setZRotation(zRotation + dz);
}

QMatrix4x4 RenderCamera::getRenderMatrix() const
{
    QMatrix4x4 mvMatrix = cameraMatrix * worldMatrix;
    mvMatrix.scale(0.05f); // make it small
    mvMatrix = projectionMatrix * mvMatrix;

    return mvMatrix;
}

QMatrix4x4 RenderCamera::getViewMatrix() const
{
    return projectionMatrix * cameraMatrix * worldMatrix;
}

void RenderCamera::renderPoint(const QVector3D& p,
                               const QColor& color,
                               float pointSize) const
{
    glPointSize(fmaxf(1.0f,pointSize));
    glBegin(GL_POINTS);
    glColor3f(color);
    glVertex3f(renderMatrix ^ p);
    glEnd();
}

void RenderCamera::renderPoint(const QVector4D& p,
                               const QColor& color,
                               float pointSize) const
{
    renderPoint(QVector3D(p[0],p[1],p[2]),color,pointSize);
}

void RenderCamera::renderLine (const QVector3D& a,
                               const QVector3D& b,
                               const QColor& color,
                               float lineWidth) const
{
    glLineWidth(fmaxf(1.0f,lineWidth));
    glBegin(GL_LINES);
    glColor4f(color);
    glVertex3f(renderMatrix ^ a);
    glVertex3f(renderMatrix ^ b);
    glEnd();
}

void RenderCamera::renderLine (const QVector4D& a,
                               const QVector4D& b,
                               const QColor& color,
                               float lineWidth) const
{
    renderLine(QVector3D(a[0],a[1],a[2]),QVector3D(b[0],b[1],b[2]),color,lineWidth);
}

void RenderCamera::renderPlane(const QVector3D& a,
                               const QVector3D& b,
                               const QVector3D& c,
                               const QVector3D& d,
                               const QColor& color,
                               float alpha) const
{
    glBegin(GL_QUADS);
    glColor4f(color,fminf(fmaxf(0.0f,alpha),1.0f));
    glVertex3f(renderMatrix ^ a);
    glVertex3f(renderMatrix ^ b);
    glVertex3f(renderMatrix ^ c);
    glVertex3f(renderMatrix ^ d);
    glEnd();
}

void RenderCamera::renderPCL  (const QVector<QVector4D>& pcl,
                               const QColor& color,
                               float pointSize) const
{
    glPointSize(fmaxf(1.0f,pointSize));
    glBegin(GL_POINTS);
    glColor3f(color);
    for (const auto & p: pcl) glVertex3f(renderMatrix ^ p);
    glEnd();
}

