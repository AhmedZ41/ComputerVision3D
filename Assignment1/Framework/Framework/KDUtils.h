#ifndef KDUTILS_H
#define KDUTILS_H

#include <QVector4D>
#include <vector>

// Sorts points by axis (0 = x, 1 = y, 2 = z)
void sortPointsByAxis(std::vector<QVector4D>& points, int axis);

// Returns the median value along the axis after sorting
float getMedianSplitValue(std::vector<QVector4D>& points, int axis);

#endif // KDUTILS_H
