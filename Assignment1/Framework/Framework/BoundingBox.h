#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <QVector4D>
#include <QVector3D>
#include <vector>
#include <utility>

std::pair<QVector3D, QVector3D> computeBoundingBox(const std::vector<QVector4D>& points);

#endif // BOUNDINGBOX_H
