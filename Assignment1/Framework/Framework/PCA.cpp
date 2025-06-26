#include "PCA.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <QDebug>
#include <iostream>  // Add this line for std::cout
#include <vector>    // Add this line for std::vector
#include <algorithm> // Add this line for std::sort



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
    
    // ADD THIS SORTING CODE HERE:
    // Sort eigenvalues and eigenvectors in descending order
    std::vector<std::pair<float, int>> eigenvalue_index_pairs;
    for (int i = 0; i < 3; ++i) {
        eigenvalue_index_pairs.push_back({eigenvalues[i], i});
    }
    std::sort(eigenvalue_index_pairs.begin(), eigenvalue_index_pairs.end(), 
              [](const auto& a, const auto& b) { return a.first > b.first; }); // Descending order

    // Reorder eigenvectors based on sorted eigenvalues
    Eigen::Matrix3f sorted_eigenvectors;
    Eigen::Vector3f sorted_eigenvalues;
    for (int i = 0; i < 3; ++i) {
        sorted_eigenvectors.col(i) = eigenvectors.col(eigenvalue_index_pairs[i].second);
        sorted_eigenvalues[i] = eigenvalue_index_pairs[i].first;
    }
    
    std::cout << "Sorted Eigenvalues: " << sorted_eigenvalues[0] << ", " << sorted_eigenvalues[1] << ", " << sorted_eigenvalues[2] << std::endl;
    std::cout << "Should be in descending order (largest to smallest)" << std::endl;

    return {sorted_eigenvectors, sorted_eigenvalues};
}
