//
// Created by badru on 19.12.2018.
//

#ifndef CODEBALLPROJECT_POINT2D_H
#define CODEBALLPROJECT_POINT2D_H
#include "math.h"

struct Point2D {
    double x {0.0};
    double z {0.0};
    Point2D() {}
    Point2D(double x, double z) : x(x), z(z) {}
    void set(double x_, double z_) { x = x_; z = z_; }
    double dist() { return sqrt(x*x + z*z); }
    Point2D normalize(double len) { return { x/len, z/len }; }
    Point2D normalize() { return { x*(1/dist()), z*(1/dist())}; }
    double distTo(double x_, double z_) { return sqrt((x-x_)*(x-x_) + (z-z_)*(z-z_)); }
    double distTo(Point2D p) { return distTo(p.x, p.z); }
    Point2D operator+(Point2D p) { return { x+p.x, z+p.z }; }
    Point2D operator-(Point2D p) { return { x-p.x, z-p.z }; }
    Point2D operator-=(Point2D p) { return { x-=p.x, z-=p.z };}
    Point2D operator*(double val) { return { x*val, z*val }; }
    void operator*=(double val) { x *= val; z *= val; }
    //TODO:???
    Point2D operator+(double val) { return { x+val, z+val }; }
    Point2D operator-(double val) { return { x-val, z-val }; }
};
#endif //CODEBALLPROJECT_POINT2D_H
