//
//  A class specializing hexahedra to cubes, i.e. eqilateral hexahedra
//
// (c) Georg Umlauf, 2021
//
#pragma once

#include "Hexahedron.h"
#include <QVector4D>

class Cube: public Hexahedron
{
private:

public:
    Cube (QVector4D _origin=E1+E0, float _sideLength=1.0);      // specialized constructor
    Cube (const Cube& c);                                       // copy-constructor
    virtual ~Cube() override {}                                 // destructor has nothing to do
};


