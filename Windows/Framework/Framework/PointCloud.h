//
// (c) Georg Umlauf, 2022
//
#pragma once

#include "SceneObject.h"
#include "RenderCamera.h"
#include "kdtree.h"

class PointCloud : public SceneObject, public QVector<QVector4D>
{
public:
    enum class TreeType
    {
        KD,
        OCT
    };

private:
    QVector3D pointsBoundMin;
    QVector3D pointsBoundMax;

    unsigned pointSize = 3;
    const float pointCloudScale = 1.5f;

    SpatialTree *spatialTree = nullptr;
    bool spatialTreeBuilt = false;

    TreeType treeType = TreeType::KD;

public:
    PointCloud();
    virtual ~PointCloud();

    bool loadPLY(const QString &);

    void buildSpatialTree();
    QVector4D findNearestPoint(const QVector4D &query);
    std::vector<QVector4D> findPointsInRadius(const QVector4D &query, float radius);

    // Get the SpatialTree for visualization
    SpatialTree *getSpatialTree();

    // Get the bounding box for visualization
    QVector3D getBoundMin() const { return pointsBoundMin; }
    QVector3D getBoundMax() const { return pointsBoundMax; }

    virtual void affineMap(const QMatrix4x4 &) override;

    virtual void draw(const RenderCamera &camera,
                      const QColor &color = COLOR_POINT_CLOUD,
                      float point_size = 3.0f) const override;
    QVector3D getMin() const { return pointsBoundMin; }
    QVector3D getMax() const { return pointsBoundMax; }

    std::vector<QVector4D> getPoints() const override;

    // setup point size
    void setPointSize(unsigned s);
    unsigned getPointSize() const { return pointSize; }

    void setTreeType(TreeType type)
    {
        treeType = type;
        spatialTreeBuilt = false;
    }
};
