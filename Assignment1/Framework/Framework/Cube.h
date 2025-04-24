#pragma once

#include "Hexahedron.h"
#include <QVector4D>

class Cube: public Hexahedron
{
public:
    Cube(const std::array<QVector4D, 8>& pts);  // New constructor

    Cube (QVector4D _origin=E1+E0, float _sideLength=1.0);
    Cube (const Cube& c);
    virtual ~Cube() override {}

    std::array<QVector4D, 8> getPoints() const {
        return Hexahedron::getPoints(); // or `return points;` if inherited
    }
};
