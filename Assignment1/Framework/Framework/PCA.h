#pragma once

#include <QVector4D>
#include <QVector>
#include <Eigen/Dense>


class PCA {
public:
    static QVector4D computeCentroid(const QVector<QVector4D>& points);
    static Eigen::Matrix3f computeCovarianceMatrix(const QVector<QVector4D>& points, const QVector4D& centroid);
    static std::pair<Eigen::Matrix3f, Eigen::Vector3f> computeEigenvectorsAndValues(const Eigen::Matrix3f& covariance);


};
