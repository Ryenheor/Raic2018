//
// Created by badru on 02.02.2019.
//

#ifndef CODEBALLPROJECT_VECT2D_H
#define CODEBALLPROJECT_VECT2D_H

#include "Point2D.h"

struct Vect2D {
    Point2D p1;
    Point2D p2;

    Vect2D(Point2D p1, Point2D p2): p1(p1),p2(p2){}
    Point2D V() { return Point2D(p2.x - p1.x, p2.z - p1.z);}
    void set(Point2D _p1, Point2D _p2) { p1 = _p1; p2 = _p2; }
    double length() { return sqrt(p1.x*p1.x + p1.z*p1.z); }
    void mult(double coeff) {
        p2.x = p1.x + V().x * coeff;
        p2.z = p1.z + V().z * coeff;
    }
    void turn(double angle)
    {
        double x = V().x * cos(angle) - V().z * sin(angle);
        double y = V().z * cos(angle) + V().x * sin(angle);
        p2.x = p1.x + x;
        p2.z = p1.z + y;
    }
};
#endif //CODEBALLPROJECT_VECT2D_H


