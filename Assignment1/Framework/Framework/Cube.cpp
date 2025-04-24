//
//  A class specializing hexahedra to cubes, i.e. eqilateral hexahedra
//
// (c) Georg Umlauf, 2021
//
#include "Cube.h"

#include "GLConvenience.h"
#include "Axes.h"

Cube::Cube(QVector4D _origin, float _sideLength):
    Hexahedron(_origin,_sideLength,_sideLength,_sideLength)
{
    type = SceneObjectType::ST_CUBE;
}

Cube::Cube (const Cube& c): Hexahedron(c)
{
    type = SceneObjectType::ST_CUBE;
}

Cube::Cube(const std::array<QVector4D, 8>& pts)
    : Hexahedron(pts)  // call the base constructor
{
    type = SceneObjectType::ST_CUBE;
}

