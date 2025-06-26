// PCA.cpp — Implementation of Principal Component Analysis for 3D point clouds

#include "PCA.h"

#include <Eigen/Dense>             // For Matrix and Vector math
#include <Eigen/Eigenvalues>       // For eigenvalue decomposition
#include <QDebug>                  // For Qt-style debug output
#include <iostream>                // For std::cout output
#include <vector>                  // For std::vector used in sorting
#include <algorithm>              // For std::sort used in eigenvalue ordering

// -----------------------------------------------------------------------------
// Function: computeCentroid
// Purpose: Computes the centroid (geometric center) of a set of 3D points.
// Why: PCA requires centering the data around the origin.
// -----------------------------------------------------------------------------
QVector4D PCA::computeCentroid(const QVector<QVector4D>& points) {
    QVector4D sum(0, 0, 0, 0);

    // Sum all points
    for (const auto& p : points) {
        sum += p;
    }

    // Divide by number of points to get average position
    if (!points.isEmpty()) {
        sum /= float(points.size());
        sum.setW(1.0f); // Ensure it's still a valid homogeneous 4D point
    }

    return sum;
}

// -----------------------------------------------------------------------------
// Function: computeCovarianceMatrix
// Purpose: Calculates the 3x3 covariance matrix from centered 3D points.
// Why: The covariance matrix captures how dimensions vary together — it's
//      the core of PCA.
// -----------------------------------------------------------------------------
Eigen::Matrix3f PCA::computeCovarianceMatrix(const QVector<QVector4D>& points, const QVector4D& centroid) {
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();

    // Accumulate outer products of (x - mean)
    for (const auto& p : points) {
        Eigen::Vector3f diff(p.x() - centroid.x(),
                             p.y() - centroid.y(),
                             p.z() - centroid.z());
        covariance += diff * diff.transpose();  // outer product
    }

    // Normalize by number of points to get average
    if (!points.isEmpty()) {
        covariance /= float(points.size());
    }

    return covariance;
}

// -----------------------------------------------------------------------------
// Function: computeEigenvectorsAndValues
// Purpose: Solves for the eigenvectors and eigenvalues of the covariance matrix.
// Why: PCA directions (axes) are eigenvectors; their importance is given by eigenvalues.
// This version also sorts them in descending order of importance (by eigenvalue).
// -----------------------------------------------------------------------------
std::pair<Eigen::Matrix3f, Eigen::Vector3f> PCA::computeEigenvectorsAndValues(const Eigen::Matrix3f& covariance) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);  // Fast, for symmetric matrices

    // Check for numerical success
    if (solver.info() != Eigen::Success) {
        qDebug() << "Eigen decomposition failed!";
        return {Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero()};
    }

    // Extract eigenvalues and eigenvectors
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();  // Columns are eigenvectors
    Eigen::Vector3f eigenvalues  = solver.eigenvalues();   // Corresponding eigenvalues

    // -----------------------------------------------------------------------------
    // Sort eigenvalues and eigenvectors in descending order of eigenvalue magnitude
    // Why: This ensures v[0] corresponds to the most significant direction of variance
    // -----------------------------------------------------------------------------

    std::vector<std::pair<float, int>> eigenvalue_index_pairs;
    for (int i = 0; i < 3; ++i) {
        eigenvalue_index_pairs.push_back({eigenvalues[i], i});
    }

    // Sort pairs based on eigenvalue, largest first
    std::sort(eigenvalue_index_pairs.begin(), eigenvalue_index_pairs.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    // Reorder eigenvectors and eigenvalues
    Eigen::Matrix3f sorted_eigenvectors;
    Eigen::Vector3f sorted_eigenvalues;
    for (int i = 0; i < 3; ++i) {
        sorted_eigenvectors.col(i) = eigenvectors.col(eigenvalue_index_pairs[i].second);
        sorted_eigenvalues[i] = eigenvalue_index_pairs[i].first;
    }

    // debug output
    std::cout << "Sorted Eigenvalues: " << sorted_eigenvalues[0] << ", "
              << sorted_eigenvalues[1] << ", " << sorted_eigenvalues[2] << std::endl;
    std::cout << "Should be in descending order (largest to smallest)" << std::endl;

    return {sorted_eigenvectors, sorted_eigenvalues};
}
