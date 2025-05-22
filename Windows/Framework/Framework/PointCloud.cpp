//
// (c) Georg Umlauf, 2022
//
#include "PointCloud.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>

#include "GLConvenience.h"
#include "QtConvenience.h"
#include "OctTree.h"
#include "KDTree.h"

using namespace std;

PointCloud::PointCloud()
{
    type = SceneObjectType::ST_POINT_CLOUD;
    pointSize = 3.0f;
}

PointCloud::~PointCloud()
{
}

bool PointCloud::loadPLY(const QString &filePath)
{
    // open stream
    fstream is;
    is.open(filePath.toStdString().c_str(), fstream::in);

    // ensure format with magic header
    string line;
    getline(is, line);
    if (line != "ply")
        throw runtime_error("not a ply file");

    // parse header looking only for 'element vertex' section size
    unsigned pointsCount = 0;
    while (is.good())
    {
        getline(is, line);
        if (line == "end_header")
        {
            break;
        }
        else
        {
            stringstream ss(line);
            string tag1, tag2, tag3;
            ss >> tag1 >> tag2 >> tag3;
            if (tag1 == "element" && tag2 == "vertex")
            {
                pointsCount = unsigned(atoi(tag3.c_str()));
            }
        }
    }

    // read and parse 'element vertex' section
    if (pointsCount > 0)
    {
        this->resize(pointsCount);
        float m = float(INT_MAX);
        pointsBoundMin = QVector3D(m, m, m);
        pointsBoundMax = -pointsBoundMin;

        stringstream ss;
        string line;
        QVector4D *p = this->data();
        for (size_t i = 0; is.good() && i < pointsCount; ++i)
        {
            getline(is, line);
            ss.str(line);
            float x, y, z;
            ss >> x >> y >> z;

            *p++ = QVector4D(x, y, z, 1.0);

            // updates for AABB
            pointsBoundMax[0] = max(x, pointsBoundMax[0]);
            pointsBoundMax[1] = max(y, pointsBoundMax[1]);
            pointsBoundMax[2] = max(z, pointsBoundMax[2]);
            pointsBoundMin[0] = min(x, pointsBoundMin[0]);
            pointsBoundMin[1] = min(y, pointsBoundMin[1]);
            pointsBoundMin[2] = min(z, pointsBoundMin[2]);
        }

        // basic validation
        if (p - this->data() < size())
            throw runtime_error("broken ply file");

        cout << "number of points: " + to_string(pointsCount) << endl;

        // rescale data
        float a, s = 0;
        for (int i = 0; i < 3; i++)
        {
            a = pointsBoundMax[i] - pointsBoundMin[i];
            s += a * a;
        }
        s = sqrt(s) / pointCloudScale;
        for (auto &p : *this)
        {
            p /= s;
            p[3] = 1.0;
        }
        //  for (int i=0; i < size(); i++) { (*this)[i]/=s; (*this)[i][3] = 1.0; }

        // Build KDTree after loading points
        buildSpatialTree();
    }
    return true;
}

void PointCloud::buildSpatialTree()
{
    if (size() == 0)
        return;
    if (isEmpty())
        return;
    QVector3D currentPointsBoundMin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    QVector3D currentPointsBoundMax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    for (const auto &p_vec : *this)
    {
        currentPointsBoundMin.setX(std::min(currentPointsBoundMin.x(), p_vec.x()));
        currentPointsBoundMin.setY(std::min(currentPointsBoundMin.y(), p_vec.y()));
        currentPointsBoundMin.setZ(std::min(currentPointsBoundMin.z(), p_vec.z()));
        currentPointsBoundMax.setX(std::max(currentPointsBoundMax.x(), p_vec.x()));
        currentPointsBoundMax.setY(std::max(currentPointsBoundMax.y(), p_vec.y()));
        currentPointsBoundMax.setZ(std::max(currentPointsBoundMax.z(), p_vec.z()));
    }
    QVector3D sizeVec = currentPointsBoundMax - currentPointsBoundMin;
    if (spatialTree)
        delete spatialTree;
    if (treeType == TreeType::OCT)
        spatialTree = new OctTree();
    else
        spatialTree = new KDTree();
    spatialTree->setBounds(currentPointsBoundMin, currentPointsBoundMax);
    std::vector<QVector4D> points;
    points.reserve(size());
    for (const auto &p : *this)
        points.push_back(p);
    spatialTree->build(points);
    spatialTree->setVisualizationLevel(3);
    spatialTreeBuilt = true;
}

SpatialTree *PointCloud::getSpatialTree()
{
    if (!spatialTreeBuilt)
        buildSpatialTree();
    return spatialTree;
}

QVector4D PointCloud::findNearestPoint(const QVector4D &query)
{
    if (!spatialTreeBuilt)
        buildSpatialTree();
    return spatialTree->findNearest(query);
}

std::vector<QVector4D> PointCloud::findPointsInRadius(const QVector4D &query, float radius)
{
    if (!spatialTreeBuilt)
        buildSpatialTree();
    return spatialTree->findInRange(query, radius);
}

void PointCloud::setPointSize(unsigned _pointSize)
{
    pointSize = _pointSize;
}

std::vector<QVector4D> PointCloud::getPoints() const
{
    return std::vector<QVector4D>(begin(), end());
}

void PointCloud::affineMap(const QMatrix4x4 &M)
{
    for (auto &p : *this)
        p = M.map(p);

    spatialTreeBuilt = false;
}

void PointCloud::draw(const RenderCamera &camera, const QColor &color, float) const
{
    camera.renderPCL((*this), color, pointSize);
}
