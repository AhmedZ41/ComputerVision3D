#include "PCA.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <QDebug>



QVector4D PCA::computeCentroid(const QVector<QVector4D>& points) {
    QVector4D sum(0, 0, 0, 0);
    for (const auto& p : points) {
        sum += p;
    }
    if (!points.isEmpty()) {
        sum /= float(points.size());
        sum.setW(1.0f); // Make it homogeneous again
    }
    return sum;
}


Eigen::Matrix3f PCA::computeCovarianceMatrix(const QVector<QVector4D>& points, const QVector4D& centroid) {
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

    for (const auto& p : points) {
        Eigen::Vector3f diff(p.x() - centroid.x(),
                             p.y() - centroid.y(),
                             p.z() - centroid.z());
        covariance += diff * diff.transpose();  // outer product
    }

    if (!points.isEmpty()) {
        covariance /= float(points.size());
    }

    return covariance;
}

std::pair<Eigen::Matrix3f, Eigen::Vector3f> PCA::computeEigenvectorsAndValues(const Eigen::Matrix3f& covariance) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);

    if (solver.info() != Eigen::Success) {
        qDebug() << "Eigen decomposition failed!";
        return {Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero()};
    }

    Eigen::Matrix3f eigenvectors = solver.eigenvectors();  // Columns are the eigenvectors
    Eigen::Vector3f eigenvalues  = solver.eigenvalues();   // Corresponding eigenvalues

    return {eigenvectors, eigenvalues};
}
