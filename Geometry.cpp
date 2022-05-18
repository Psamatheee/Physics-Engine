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
    if(shape.get_type() == Type::AABB){
        Circle c{radius,centre};
        return shape.intersects(c);
    }
    return false;
}

Boundary Circle::collides_boundary(double w, double h) {

    if(get_position().get_y() + get_radius() > h){
        if(get_position().get_x() + get_radius() >= w  ) return Boundary::TR;
        if(get_position().get_x() - get_radius() <= 0  ) return Boundary::TL;
        return Boundary::Top;
    }
    if( get_position().get_y() - get_radius() <= 0  ){
        if(get_position().get_x() + get_radius() >= w) return Boundary::BR;
        if(get_position().get_x() - get_radius() <= 0) return Boundary::BL;
        return Boundary::Bottom;
    }
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
Vec get_closest_point(Rectangle& r, Circle& c){
    Vec centre = c.get_position();
    double x = centre.get_x();
    double y = centre.get_y();
    double eeee = 0;
    if(x < r.max.get_x() && x>r.min.get_x() && y > r.min.get_y() && y < r.max.get_y()){
       double up = r.max.get_y() - y;
       double down = y - r.min.get_y();
       double left = x-r.min.get_x();
       double right = r.max.get_x()- x;
       if(up < down && up < left && up < right){
           return Vec{x,r.max.get_y()};
       }else if( down < left && down < right && down < up){
           return Vec{x,r.min.get_y()};
       }else if(left < down && left < right && left<up){
           return Vec{r.min.get_x(), y};
       }else{
           return Vec{r.max.get_x(),y};
       }
    }
    if(x < r.min.get_x()){
        if(y>r.max.get_y()){
            return Vec{r.min.get_x(), r.max.get_y()};
        } else if(y < r.min.get_y()){
            return Vec{r.min.get_x(),r.min.get_y()};
        }else{
            return Vec{r.min.get_x(),y};
        }
    }else if(x > r.max.get_x()){
        if(y>r.max.get_y()){
            return Vec{r.max.get_x(),r.max.get_y()};
        }else if(y < r.min.get_y()){
            return Vec{r.max.get_x(),r.min.get_y()};
        }else{
            return Vec{r.max.get_x(),y};
        }
    }else{
        if(y>r.max.get_y()){
            return Vec{x,r.max.get_y()};
        }else{
            return Vec{x,r.min.get_y()};
        }
    }
}
void AABB::set_position(double xx, double yy) {
    double width = max.get_x() - min.get_x();
    double height = max.get_y() - min.get_y();
  //  std::cout<<width << " "<<height<<"\n";
    min.set_x(xx - width);
    min.set_y(yy-height);
    max.set_y(yy);
    max.set_x(xx);
}

bool AABB::intersects(Shape& shape){
    if(shape.get_type() == Type::AABB){
        if (min.get_x() > shape.get_max().get_x() || max.get_x() < shape.get_min().get_x()) return false;
        // check if there is separation along the y-.get_x()is
        if (min.get_y() > shape.get_max().get_y()|| max.get_y() < shape.get_min().get_y()) return false;

        std::cout << "inter\n";
        return true;
    }
    if(shape.get_type() == Type::Circle){
        Rectangle r = Rectangle{min,max};
        Circle c{shape.get_radius(),shape.get_position() };
        Vec closest = get_closest_point(r, c);
        double distance_sqr = pow(closest.get_x() - shape.get_x(),2) + pow(closest.get_y() - shape.get_y(),2);
        return pow( c.get_radius(),2) >= distance_sqr;
    }
    return false;

}

Boundary AABB::collides_boundary(double w, double h) {
    if(max.get_y() > h){
        if(max.get_x() > w  ) return Boundary::TR;
        if(min.get_x() < 0) return Boundary::TL;
        return Boundary::Top;
    }
    if( min.get_y() < 0  ){
        if(max.get_x() > w) return Boundary::BR;
        if(min.get_x() < 0) return Boundary::BL;
        return Boundary::Bottom;
    }


    if(max.get_x() > w ) return Boundary::Right;
    if(min.get_x() < 0) return Boundary::Left;
    return Boundary::None;
}

bool intersects(Shape& a, Shape& b){
    if(a.get_type() == Type::AABB && b.get_type() == Type::AABB){

    }
}
