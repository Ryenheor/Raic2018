//
// Created by badru on 19.12.2018.
//

#ifndef CODEBALLPROJECT_POINT3D_H
#define CODEBALLPROJECT_POINT3D_H

#include "Point2D.h"

struct Point3D : Point2D {
    double y {0.0}; // высота
    Point3D() {}
    Point3D(double x, double z, double y) : y(y) { Point2D::set(x, z); }
    void set(double x_, double z_, double y_) { x = x_; z = z_; y = y_; }
    double dist() { return sqrt(x*x + y*y + z*z); }
    double distTo(double x_, double z_, double y_) { return sqrt((x-x_)*(x-x_) + (y-y_)*(y-y_) + (z-z_)*(z-z_)); }
    double distTo(Point3D p) { return distTo(p.x, p.z, p.y); }
    Point3D operator+(Point3D p) { return { x+p.x, z+p.z, y+p.y };}
    Point3D operator-(Point3D p) { return { x-p.x, z-p.z, y-p.y };}
    Point3D operator+=(Point3D p) { return { x+=p.x, z+=p.z, y+=p.y };}
    Point3D operator-=(Point3D p) { return { x-=p.x, z-=p.z, y-=p.y };}
    Point3D operator*(double val) { return { x*val, z*val, y*val }; }
    void operator*=(double val) { x *= val; z *= val; y *= val; }
    Point3D normalize() { return { x*(1/dist()), z*(1/dist()), y*(1/dist()) }; }
};
#endif //CODEBALLPROJECT_POINT3D_H
