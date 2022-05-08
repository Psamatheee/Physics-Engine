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

Vec operator*(float num, Vec &v) {
    return Vec{v.get_x() * num, v.get_y() * num};
}

//Vector helper functions
float dotProd(Vec& v1, Vec& v2){
    return v1.get_x() * v2.get_x() + v1.get_y() * v2.get_y();
}

Vec scalar_mult(float num, Vec& v){
    return {v.get_x()*num, v.get_y()*num};
}

//====================================================================================================================
//Circle Functions

bool Circle::intersects(Shape &shape) {
    if(shape.get_type() == Type::Circle){
        float distance_sqr = powf(centre.get_x() - shape.get_x(),2) + powf(centre.get_y() - shape.get_y(),2);
        return powf(radius + shape.get_radius(),2) >= distance_sqr;
    }
    return false;
}

Boundary Circle::collides_boundary(float w, float h) {
    if(get_position().get_y() + get_radius() >= h) return Boundary::Top;
    if( get_position().get_y() - get_radius() <= 0  ) return Boundary::Bottom;
    if(get_position().get_x() + get_radius() >= w  ) return Boundary::Right;
    if(get_position().get_x() - get_radius() <= 0  ) return Boundary::Left;
    return Boundary::None;
}

bool does_circle_intersect(Circle& c1, Circle& c2){
    //using sqr instead of sqrt as it's more efficient
    float distance_sqr = powf(c1.get_x() - c2.get_x(),2) + powf(c1.get_y() - c2.get_y(),2);
    return powf(c1.get_radius() + c2.get_radius(),2) >= distance_sqr;
}

//get how far in the circles are intersecting each other
float get_depth(Shape& a, Shape& b) {
    if(a.get_type() == Type::Circle && b.get_type() == Type::Circle){
        float distance_sqr = powf(a.get_x() - b.get_x(), 2) +
                             powf(a.get_y() - b.get_y(), 2);
        float distance = sqrtf(distance_sqr);
        return a.get_radius() + b.get_radius() - distance;
    }
    return 0;

}
