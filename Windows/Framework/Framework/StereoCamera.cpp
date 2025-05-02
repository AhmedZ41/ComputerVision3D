#include "StereoCamera.h"
#include "Hexahedron.h"
#include <cmath>


StereoCamera::StereoCamera(const PerspectiveCamera& left,
                           const PerspectiveCamera& right)
    : leftCam(left), rightCam(right)
{
    type = SceneObjectType::ST_STEREO_CAMERA;

    //misalignRightCamera(1.0f);
}

void StereoCamera::misalignRightCamera(float angleInDegrees)
{
    // Create a rotation matrix for the misalignment around the Y axis
    QMatrix4x4 misalignmentRotation;
    misalignmentRotation.setToIdentity();
    misalignmentRotation.rotate(angleInDegrees, 0, 1, 0); // Rotate around Y-axis

    // Apply misalignment to right camera's rotation
    QMatrix4x4 currentRotation = rightCam.getRotation();
    rightCam.setRotation(currentRotation * misalignmentRotation);
}


void StereoCamera::affineMap(const QMatrix4x4& matrix)
{
    leftCam.affineMap(matrix);
    rightCam.affineMap(matrix);
    for (auto& objPts : reconstructedPoints)
        for (auto& p : objPts)
            p = matrix * p;
}

void StereoCamera::draw(const RenderCamera& renderer,
                        const QColor&       /*color*/,
                        float               lineWidth) const
{
    leftCam.draw(renderer, QColor(255, 0, 0), lineWidth);
    rightCam.draw(renderer, QColor(0, 255, 255), lineWidth);

    // points and edges
    for (size_t h = 0; h < hexes.size(); ++h) {
        const auto* hex = hexes[h];
        const auto& pts = reconstructedPoints[h];
        auto originalPts = hex->getPoints();

        for (size_t i = 0; i < pts.size(); ++i) {
            const auto& p4 = pts[i];
            if (p4.w() == 1.0f) {
                // Check if we have error data stored in w component
                float errorFactor = (p4.w() > 1.0f) ? (p4.w() - 1.0f) : 0.0f;
                errorFactor = std::min(errorFactor * 5.0f, 1.0f); // Scale for visibility

                // Color gradient from magenta (low error) to red (high error)
                QColor pointColor = QColor(
                    255,  // R
                    0,    // G
                    std::max(0, int(255 * (1.0f - errorFactor)))  // B (fades as error increases)
                    );
                renderer.renderPoint(p4, pointColor, 4.0f);
            }
        }

        // draw edges of the reconstruction
        auto edges = hex->getEdgeIndices();
        for (auto [i0, i1] : edges) {
            if (i0 < pts.size() && i1 < pts.size() &&
                pts[i0].w() >= 1.0f && pts[i1].w() >= 1.0f) {
                QVector3D a = pts[i0].toVector3D();
                QVector3D b = pts[i1].toVector3D();
                renderer.renderLine(a, b, Qt::white, 5.0f);
            }
        }
    }
}

std::vector<QVector4D> StereoCamera::getPoints() const
{
    std::vector<QVector4D> flat;
    for (const auto& objPts : reconstructedPoints)
        for (const auto& p4 : objPts)
            if (p4.w() >= 1.0f)
                flat.push_back(p4);
    return flat;
}

void StereoCamera::reconstruct(const std::vector<SceneObject*>& scene)
{
    reconstructedPoints.clear();
    hexes.clear();

    // precompute camera geometry
    QVector3D oL = leftCam.getCenter().toVector3D();
    QMatrix4x4 RL = leftCam.getRotation();
    float fL = leftCam.getFocalLength();
    float halfL = leftCam.getPlaneSize();
    QVector3D cxL = RL.column(0).toVector3D().normalized();
    QVector3D cyL = RL.column(1).toVector3D().normalized();
    QVector3D czL = -RL.column(2).toVector3D().normalized();
    QVector3D HL = oL + czL * fL;

    QVector3D oR = rightCam.getCenter().toVector3D();
    QMatrix4x4 RR = rightCam.getRotation();
    float fR = rightCam.getFocalLength();
    float halfR = rightCam.getPlaneSize();
    QVector3D cxR = RR.column(0).toVector3D().normalized();
    QVector3D cyR = RR.column(1).toVector3D().normalized();
    QVector3D czR = -RR.column(2).toVector3D().normalized();
    QVector3D HR = oR + czR * fR;

    // baseline
    float baseline = QVector3D::dotProduct(oR - oL, cxL);
    const float depthScale = 0.4f;

    // Max error threshold for coloring
    const float maxErrorThreshold = 0.1f;

    // for each hexahedron
    for (auto* obj : scene) {
        auto* hex = dynamic_cast<const Hexahedron*>(obj);
        if (!hex) continue;
        hexes.push_back(hex);
        auto pts4 = hex->getPoints();
        std::vector<QVector4D> ptsRe(pts4.size(), QVector4D(0,0,0,0));

        for (size_t i = 0; i < pts4.size(); ++i) {
            // project onto left and right image planes
            QVector3D qL = leftCam.projectPoint(pts4[i]);
            QVector3D qR = rightCam.projectPoint(pts4[i]);
            // compute image coordinates
            QVector2D uvL(
                QVector3D::dotProduct(qL - HL, cxL),
                QVector3D::dotProduct(qL - HL, cyL)
                );
            QVector2D uvR(
                QVector3D::dotProduct(qR - HR, cxR),
                QVector3D::dotProduct(qR - HR, cyR)
                );
            // ensure within both view frustums
            if (std::fabs(uvL.x()) > halfL || std::fabs(uvL.y()) > halfL ||
                std::fabs(uvR.x()) > halfR || std::fabs(uvR.y()) > halfR) {
                continue;
            }

            // For Part 3: Calculate y-parallax (vertical disparity)
            // In a perfect normal case, this should be zero
            // With camera misalignment, this will be non-zero
            float yParallax = std::fabs(uvL.y() - uvR.y());

            // disparity (x-parallax)
            float disp = uvL.x() - uvR.x();
            if (std::fabs(disp) < 1e-6f) continue;

            // reconstruct depth
            float z = -fL * baseline / disp;
            z *= depthScale;
            float x = -z * uvL.x() / fL;
            float y = -z * uvL.y() / fL;

            // Calculate reconstruction from both cameras
            QVector3D PwL = oL + RL.map(QVector3D(x,y,z));

            // Calculate from right camera view too
            float xR = -z * uvR.x() / fR;
            float yR = -z * uvR.y() / fR;
            QVector3D PwR = oR + RR.map(QVector3D(xR,yR,z));

            // Calculate error as distance between two reconstructions (due to misalignment)
            float reconstructionError = (PwL - PwR).length();

            // Also incorporate y-parallax as part of error metric
            float totalError = reconstructionError + yParallax;
            totalError = std::min(totalError, maxErrorThreshold) / maxErrorThreshold;

            // Average to place the point between cameras
            QVector3D Pw = (PwL + PwR) * 0.5f;

            // Store the point with error factor encoded in w component
            // w = 1.0 + error factor (where error factor is normalized 0-1)
            ptsRe[i] = QVector4D(Pw, 1.0f + totalError);
        }
        reconstructedPoints.push_back(std::move(ptsRe));
    }
}

QVector3D StereoCamera::triangulate(const QVector3D& o1, const QVector3D& d1,
                                    const QVector3D& o2, const QVector3D& d2) const
{
    QVector3D r = o2 - o1;
    float a = QVector3D::dotProduct(d1,d1);
    float b = QVector3D::dotProduct(d1,d2);
    float c = QVector3D::dotProduct(d2,d2);
    float d = QVector3D::dotProduct(d1,r);
    float e = QVector3D::dotProduct(d2,r);
    float den = a*c - b*b;
    if (std::fabs(den) < 1e-6f) return 0.5f*(o1+o2);
    float t1 = (d*c - b*e)/den;
    float t2 = (a*e - b*d)/den;
    QVector3D p1 = o1 + t1*d1;
    QVector3D p2 = o2 + t2*d2;

    // Calculate error for Part 3 - represents triangulation error due to misalignment
    float error = (p1 - p2).length();

    // For Part 3: We could print the error to the console for debugging
    if (error > 0.01f) {
        // This would log larger triangulation errors, showing the impact of misalignment
        // But we'll leave it commented out to avoid console spam
        // std::cout << "Triangulation error: " << error << std::endl;
    }

    return 0.5f*(p1+p2);
}

void StereoCamera::drawProjections(const RenderCamera&              renderer,
                                   const std::vector<SceneObject*>& scene) const
{
    // Use different colors for left and right camera projections
    // This helps visualize the misalignment in Part 3
    leftCam.drawProjections(renderer, scene, Qt::red);
    rightCam.drawProjections(renderer, scene, Qt::cyan);
}
