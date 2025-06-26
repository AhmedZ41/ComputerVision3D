#ifndef SPATIALTREE_H
#define SPATIALTREE_H

#include "SceneObject.h"
#include <vector>
#include <QVector3D>
#include <QVector4D>
#include <QColor>
class RenderCamera;

// Abstract base class for spatial trees
class SpatialTree : public SceneObject
{
public:
    virtual ~SpatialTree() {}
    // Build the tree from a set of points
    virtual void build(const std::vector<QVector4D> &points) = 0;
    // Set the bounding box for visualization
    virtual void setBounds(const QVector3D &min, const QVector3D &max) = 0;
    // Set the visualization level (e.g., max depth to visualize)
    virtual void setVisualizationLevel(int level) = 0;
    // Find the nearest point to a query
    virtual QVector4D findNearest(const QVector4D &query) = 0;
    // Find all points within a radius of a query
    virtual std::vector<QVector4D> findInRange(const QVector4D &query, float radius) = 0;
    // Check if the tree is empty
    virtual bool isEmpty() const = 0;
    // Apply an affine transformation to the tree
    virtual void affineMap(const QMatrix4x4 &matrix) override = 0;
    // Draw the tree visualization
    virtual void draw(const RenderCamera &renderer, const QColor &color = QColor(0, 200, 255), float transparency = 0.4f) const override = 0;
    // Get all points in the tree
    virtual std::vector<QVector4D> getPoints() const override = 0;

protected:
    // Helper function to get line width based on level
    float getLevelLineWidth(int level) const
    {
        float baseWidth = 5.0f;
        float decayRate = 0.3f;
        float width = baseWidth * std::exp(-decayRate * level);
        return std::max(width, 1.0f);
    }
    // Helper function to get color based on level
    QColor getLevelColor(int level) const
    {
        // Generate a unique color for each level using HSV color wheel
        // Hue cycles through 0-359, saturation and value are fixed for visibility
        int hue = (level * 47) % 360;         // 47 is a prime to avoid repetition for small levels
        int sat = 200;                        // out of 255
        int val = 220;                        // out of 255
        int alpha = 120 + (level * 20) % 136; // Vary alpha for some transparency
        return QColor::fromHsv(hue, sat, val, alpha);
    }
};

// Box structure for spatial partitioning visualization
struct SpatialPartitionBox
{
    QVector4D center;     // Center point of the box
    QVector3D dimensions; // Width, height, depth of the box
    int level;            // Tree level (for visualization)
};

#endif // SPATIALTREE_H
