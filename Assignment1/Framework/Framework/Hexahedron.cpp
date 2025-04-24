//
//  A simple class for hexahedral objects
//
// (c) Georg Umlauf, 2021
//
#include "Hexahedron.h"
#include "QtConvenience.h"

Hexahedron::Hexahedron(QVector4D _origin,
                       float     _dx,
                       float     _dy,
                       float     _dz):
    origin(_origin), dx(_dx), dy(_dy), dz(_dz)
{
    type = SceneObjectType::ST_HEXAHEDRON;

    QVector3D o(origin.x(),origin.y(), origin.z());
    this->push_back(o);
    this->push_back(o+QVector3D( 0,dy, 0));
    this->push_back(o+QVector3D( 0,dy,dz));
    this->push_back(o+QVector3D( 0, 0,dz));
    this->push_back(o+QVector3D(dx, 0, 0));
    this->push_back(o+QVector3D(dx,dy, 0));
    this->push_back(o+QVector3D(dx,dy,dz));
    this->push_back(o+QVector3D(dx, 0,dz));
}

Hexahedron::Hexahedron(const Hexahedron& hex):
    SceneObject(hex),
    std::vector<QVector3D>(hex)
{
    origin = hex.origin;
    dx     = hex.dx;
    dy     = hex.dy;
    dz     = hex.dz;
    type   = SceneObjectType::ST_HEXAHEDRON;
}

Hexahedron::Hexahedron(const Hexahedron&& hex):
    SceneObject(hex),
    std::vector<QVector3D>(hex)
{
    origin = hex.origin;
    dx     = hex.dx;
    dy     = hex.dy;
    dz     = hex.dz;
    type   = SceneObjectType::ST_HEXAHEDRON;
}

void Hexahedron::affineMap(const QMatrix4x4& M)
{
    origin = M*origin;
    for (auto& p: *this) p = M^p;
}

void Hexahedron::draw(const RenderCamera& renderer,
                      const QColor& color,
                      float lineWidth) const
{
    for (unsigned i=0; i<2*edgeCount; i+=2)
        renderer.renderLine(
            (*this)[edgeList[i  ]],
            (*this)[edgeList[i+1]],
            color,
            lineWidth);
}

void Hexahedron::drawPoints(const RenderCamera& renderer,
                            const QColor& col,
                            float pointSize) const
{
    for (auto p: *this) renderer.renderPoint(p,col,pointSize);
}

QDebug& operator << (QDebug& dbg, const Hexahedron &hex)
{
    dbg.nospace() << "Hexahedron:";
    for (auto p: hex) dbg.nospace() << p;
    dbg.nospace() << "\n";
    return dbg.space();
}




Hexahedron::Hexahedron(const std::array<QVector4D, 8>& pts)
{
    for (const auto& p : pts)
        this->push_back(QVector3D(p.x(), p.y(), p.z()));  // convert 4D to 3D and store in inherited vector
}

