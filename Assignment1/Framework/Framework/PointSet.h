#ifndef POINTSET_H
#define POINTSET_H

#include <QVector4D>
#include <QVector3D>
#include <vector>
#include <utility>

class PointSet {
public:
    // Constructors
    PointSet(); // empty
    PointSet(const std::vector<QVector4D>& points);

    // Accessors
    const std::vector<QVector4D>& getPoints() const;
    const QVector3D& getMinCorner() const;
    const QVector3D& getMaxCorner() const;

    // Utility
    size_t size() const;
    bool empty() const;

    PointSet extractSubsetInside(const QVector3D& minCorner, const QVector3D& maxCorner) const;


private:
    std::vector<QVector4D> points;
    QVector3D minCorner;
    QVector3D maxCorner;

    void computeBoundingBox(); // called internally
};

#endif // POINTSET_H
