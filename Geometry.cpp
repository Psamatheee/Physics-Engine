//
// Created by julia on 08/05/22.
//

#include "Geometry.h"
void Vec::normalize() {
    x = x/size;
    y = y/size;
}

Vec operator-(Vec& v1, Vec& v2){
    Vec v{v1.x - v2.x, v1.y - v2.y};
    return v;
}

Vec operator+(Vec& v1, Vec& v2){
    Vec v{v1.x + v2.x, v1.y + v2.y};
    return v;
}

Vec operator*(double num, Vec &v) {
    return Vec{v.get_x() * num, v.get_y() * num};
}

double Vec::angle_from_xaxis() {
    return std::atan(y/x);
}

//Vector helper functions
double dotProd(Vec& v1, Vec& v2){
    return v1.get_x() * v2.get_x() + v1.get_y() * v2.get_y();
}

Vec scalar_mult(double num, Vec& v){
    return {v.get_x()*num, v.get_y()*num};
}

//====================================================================================================================
//Circle Functions

bool Circle::intersects(Shape &shape) {
    if(shape.get_type() == Type::Circle){
        //Circle circ = shape;
        //shape.get_centre();
        double distance_sqr = pow(centre.get_x() - shape.get_x(),2) + pow(centre.get_y() - shape.get_y(),2);
        return pow(radius + shape.get_radius(),2) >= distance_sqr;
    }
    return false;
}

Boundary Circle::collides_boundary(double w, double h) {
    if(get_position().get_y() + get_radius() >= h) return Boundary::Top;
    if( get_position().get_y() - get_radius() <= 0  ) return Boundary::Bottom;
    if(get_position().get_x() + get_radius() >= w  ) return Boundary::Right;
    if(get_position().get_x() - get_radius() <= 0  ) return Boundary::Left;
    return Boundary::None;
}

Rectangle Circle::get_bounding_box() {
    Rectangle bb;
    bb.min.set_x( get_position().get_x() - radius);
    bb.min.set_y( get_position().get_y() - radius);
    bb.max.set_x( get_position().get_x() + radius);
    bb.max.set_y( get_position().get_y() + radius);
    return bb;
}

bool does_circle_intersect(Circle& c1, Circle& c2){
    //using sqr instead of sqrt as it's more efficient
    double distance_sqr = pow(c1.get_x() - c2.get_x(),2) + pow(c1.get_y() - c2.get_y(),2);
    return pow(c1.get_radius() + c2.get_radius(),2) >= distance_sqr;
}

//get how far in the circles are intersecting each other
double get_depth(Shape& a, Shape& b) {
    if(a.get_type() == Type::Circle && b.get_type() == Type::Circle){
        double distance_sqr = pow(a.get_x() - b.get_x(), 2) +
                             pow(a.get_y() - b.get_y(), 2);
        double distance = sqrt(distance_sqr);
        return a.get_radius() + b.get_radius() - distance;
    }
    return 0;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AABB functions

void AABB::set_position(double xx, double yy) {
    double width = max.get_x() - min.get_x();
    double height = max.get_y() - max.get_y();
    max.set_x(xx + width);
    max.set_y(yy+height);
    min.set_y(yy);
    min.set_x(xx);
}

bool AABB::intersects(Shape& shape){
    if(shape.get_type() == Type::AABB){
        if (min.get_x() > shape.get_max().get_x() || max.get_x() < shape.get_min().get_x()) return false;
        // check if there is separation along the y-.get_x()is
        if (min.get_y() > shape.get_max().get_x()|| max.get_x() < shape.get_min().get_y()) return false;
        return true;
    }
    return false;

}

Boundary AABB::collides_boundary(double w, double h) {
    if(max.get_y() > h || min.get_y() > h) return Boundary::Top;
    if(min.get_y() < 0) return Boundary::Bottom;
    if(max.get_x() > w ) return Boundary::Right;
    if(min.get_x() < 0) return Boundary::Left;
    return Boundary::None;
}

bool intersects(Shape& a, Shape& b){
    if(a.get_type() == Type::AABB && b.get_type() == Type::AABB){

    }
}
