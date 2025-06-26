#include "PCAAxes.h"

PCAAxes::PCAAxes(const QVector4D& centroid, const Eigen::Matrix3f& eigenvectors, float scale)
    : origin(centroid.toVector3D()), scale(scale)
{
    for (int i = 0; i < 3; ++i) {
        directions[i] = QVector3D(eigenvectors(0, i), eigenvectors(1, i), eigenvectors(2, i));
    }
    type = SceneObjectType::ST_PCA_AXES;
}

void PCAAxes::draw(const RenderCamera& renderer, const QColor& color, float lineWidth) const {
    // Use a much larger scaling factor for visibility
    float axisLength = 0.5f;  // Increased from smaller value to make axes much longer
    
    // Draw each principal component axis with much increased thickness
    for (int i = 0; i < 3; ++i) {
        QVector4D direction = QVector4D(directions[i], 0.0f);
        QVector4D originPoint = QVector4D(origin, 1.0f);  // Convert QVector3D to QVector4D
        QVector4D endPoint = originPoint + axisLength * direction;
        
        // Use different colors for each axis for better distinction
        QColor axisColor;
        switch(i) {
            case 0: axisColor = QColor(255, 0, 0);   break; // Red for 1st principal component
            case 1: axisColor = QColor(0, 255, 0);   break; // Green for 2nd principal component  
            case 2: axisColor = QColor(0, 0, 255);   break; // Blue for 3rd principal component
        }
        
        // Draw with much increased line width (multiply by 4 for extra thickness)
        renderer.renderLine(QVector3D(originPoint), QVector3D(endPoint), axisColor, lineWidth * 4.0f);
    }
}

void PCAAxes::affineMap(const QMatrix4x4& m) {
    origin = m * origin;
    for (auto& dir : directions)
        dir = m.mapVector(dir);
}
