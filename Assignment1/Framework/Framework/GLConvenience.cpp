//
//  Some convenience functions for OpenGL-/Qt-conversions
//
// (c) Georg Umlauf, 2021
//
#include "GLConvenience.h"

void glVertex3f(const QVector3D& vector)
{
    glVertex3f(vector.x(),vector.y(),vector.z());
}

void glVertex3f(const QVector4D& vector)
{
    glVertex3f(vector.x(),vector.y(),vector.z());
}

void glColor3f (const QColor& color)
{
    glColor3f (float(color.red  ())/255.0f,
               float(color.green())/255.0f,
               float(color.blue ())/255.0f  );
}

void glColor4f (const QColor& color)
{
    glColor4f (float(color.red  ())/255.0f,
               float(color.green())/255.0f,
               float(color.blue ())/255.0f,
               float(color.alpha())/255.0f);
}

void glColor4f (const QColor& color, float transpareny)
{
    glColor4f (float(color.red  ())/255.0f,
               float(color.green())/255.0f,
               float(color.blue ())/255.0f,
               transpareny);
}
