//
// Created by julia on 08/05/22.
//

#include <cfloat>
#include "Geometry.h"
#include <iostream>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//VECTOR FUNCTIONS

double Vec::get_size()  {
    if(size == 1) return 1;
    size =  sqrt(x * x + y * y);
    return size;
}

void Vec::normalize() {
    get_size();
    if(size == 0) {
        x = 0;
        y = 0;
        return;
    }

    x = x / size;
    y = y / size;
    size = 1;
}

Vec operator-(const Vec &v1, const Vec &v2) {
    Vec v{v1.x - v2.x, v1.y - v2.y};
    return v;
}

Vec operator+(const Vec &v1, const Vec &v2) {
    Vec v{v1.x + v2.x, v1.y + v2.y};
    return v;
}

Vec operator*(double num, const Vec &v) {
    return Vec{v.x * num, v.y * num};
}
bool operator==(Vec v1, Vec v2){
    if(v1.x == v2.x && v1.y == v2.y) return true;
    return false;
}
Vec Vec::orthogonalize() const {
    Vec v  =  Vec{y,-x};
    v.normalize();
    return v;
}

Vec Vec::rotate(double angle) {
    Matrix matrix = get_rotation_m(angle);
    return matrix * this;
}

double dotProd(const Vec &v1, const Vec &v2) {
    return v1.x * v2.x + v1.y * v2.y;
}
double cross(Vec a, Vec b){
    return a.x*b.y - a.y*b.x;
}
Vec cross( const Vec& a, double s )
{
    return Vec{s * a.y, -s * a.x };
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MATRIX FUNCTIONS

Matrix get_rotation_m(double angle) {
    double cos = std::cos((angle));
    double sin = std::sin(angle);

    return Matrix{cos, sin, -sin, cos};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CIRCLE FUNCTIONS

void Circle::rotate(double angle) {
orient += angle;
if (orient > 2 * M_PI) orient = orient - 2 * M_PI;
if (orient < 0) orient = 2 * M_PI + orient;
};

bool Circle::intersects(Shape &shape) {
    if (shape.get_type() == Type::Circle) {
        double distance_sqr = pow(centre.x - shape.get_position().x, 2) + pow(centre.y - shape.get_position().y, 2);
        return pow(radius + shape.get_radius(), 2) >= distance_sqr;
    }
    if (shape.get_type() == Type::AABB) {
        return shape.intersects(*this);
    }
    if (shape.get_type() == Type::OBB) {
        return shape.intersects(*this);
    }
    return false;
}

double Circle::get_inertia(double mass){
    return  1.0/2 * mass * radius * radius;
}

Rectangle Circle::get_bounding_box() {
    Rectangle bb;
    bb.min.x = get_position().x - radius;
    bb.min.y = get_position().y - radius;
    bb.max.x = get_position().x + radius;
    bb.max.y = get_position().y + radius;
    return bb;
}

//get how far in the circles are intersecting each other
double get_depth(Shape &a, Shape &b) {
    if (a.get_type() == Type::Circle && b.get_type() == Type::Circle) {
        double distance_sqr = pow(a.get_position().x - b.get_position().x, 2) +
                              pow(a.get_position().y - b.get_position().y, 2);
        double distance = sqrt(distance_sqr);
        return a.get_radius() + b.get_radius() - distance;
    }
    return 0;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AABB functions
Vec get_closest_point(Rectangle &r, Circle &c) {
    Vec centre = c.get_position();
    double x = centre.x;
    double y = centre.y;
    if (x < r.max.x && x > r.min.x && y > r.min.y && y < r.max.y) {
        double up = r.max.y - y;
        double down = y - r.min.y;
        double left = x - r.min.x;
        double right = r.max.x - x;
        if (up < down && up < left && up < right) {
            return Vec{x, r.max.y};
        } else if (down < left && down < right && down < up) {
            return Vec{x, r.min.y};
        } else if (left < down && left < right && left < up) {
            return Vec{r.min.x, y};
        } else {
            return Vec{r.max.x, y};
        }
    }
    if (x < r.min.x) {
        if (y > r.max.y) {
            return Vec{r.min.x, r.max.y};
        } else if (y < r.min.y) {
            return Vec{r.min.x, r.min.y};
        } else {
            return Vec{r.min.x, y};
        }
    } else if (x > r.max.x) {
        if (y > r.max.y) {
            return Vec{r.max.x, r.max.y};
        } else if (y < r.min.y) {
            return Vec{r.max.x, r.min.y};
        } else {
            return Vec{r.max.x, y};
        }
    } else {
        if (y > r.max.y) {
            return Vec{x, r.max.y};
        } else {
            return Vec{x, r.min.y};
        }
    }
}

void AABB::set_position(double xx, double yy) {
    double width = max.x - min.x;
    double height = max.y - min.y;
    min.x = xx - width;
    min.y = yy - height;
    max.y = yy;
    max.x = xx;
}


double AABB::get_inertia(double mass){
    return 0;
}
bool AABB::intersects(Shape &shape) {
    if (shape.get_type() == Type::AABB) {
        if (min.x > shape.get_max().x || max.x < shape.get_min().x) return false;
        // check if there is separation along the y-.xis
        if (min.y > shape.get_max().y || max.y < shape.get_min().y) return false;

        std::cout << "inter\n";
        return true;
    }
    if (shape.get_type() == Type::Circle) {
        Rectangle r = Rectangle{min, max};
        Vec pos = shape.get_position();
        bool inside = pos.x > min.x && pos.x < max.x && pos.y < max.y && pos.y > min.y;
        if(inside) return true;
        Circle c{shape.get_radius(), shape.get_position()};
        Vec closest = get_closest_point(r, c);
        double distance_sqr = pow(closest.x - shape.get_position().x, 2) + pow(closest.y - shape.get_position().y, 2);
        return pow(c.get_radius(), 2) >= distance_sqr;
    }
    if (shape.get_type() == Type::OBB) {

        //AABB normals
        Vec n1{0, 1};
        Vec n0{1, 0};

        //OBB normals
        Vec n2 = shape.get_max();
        Vec n3 = shape.get_min();
        n2.normalize();
        n3.normalize();

        double h = (max.y - min.y);
        //go through all normals to check if there's a separation axis
        //n0
        double amax = max.x;
        double amin = min.x;

        Helper_Rect rect = shape.get_points();
        double bmin = dotProd(rect[0], n0);
        double bmax = dotProd(rect[0], n0);
        for (int i = 0; i < 4; i++) {
            Vec point = rect[i];
            double distance = dotProd(point, n0);
            if (distance > bmax) bmax = distance;
            if (distance < bmin) bmin = distance;


        }
        if (amin - bmax > 0 || bmin - amax > 0) return false; // separation along yaxis;

        //n1
        amax = max.y;
        amin = min.y;
        bmin = dotProd(rect[0], n1);
        bmax = dotProd(rect[0], n1);
        for (int i = 0; i < 4; i++) {
            Vec point = rect[i];
            double distance = dotProd(point, n1);
            if (distance > bmax) bmax = distance;
            if (distance < bmin) bmin = distance;
        }
        if (amin - bmax > 0 || bmin - amax > 0) return false; // separation along xaxis;

        //n2
        bmax = dotProd(rect.point1, n2);
        bmin = dotProd(rect.point2, n2);
        if (bmin > bmax) {
            double temp = bmin;
            bmin = bmax;
            bmax = temp;
        }
        amin = dotProd(max, n2);
        amax = amin;
        double temp = dotProd(Vec{max.x, max.y - h}, n2);
        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;
        temp = dotProd(min, n2);

        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;
        temp = dotProd(Vec{min.x, min.y + h}, n2);
        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;

        if (amin - bmax > 0 || bmin - amax > 0) return false; // separation along n3

        //n3
        bmax = dotProd(rect.point1, n3);
        bmin = dotProd(rect.point3, n3);
        if (bmin > bmax) {
            temp = bmin;
            bmin = bmax;
            bmax = temp;
        }
        amin = dotProd(max, n3);
        amax = amin;
        temp = dotProd(Vec{max.x, max.y - h}, n3);
        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;
        temp = dotProd(min, n3);

        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;
        temp = dotProd(Vec{min.x, min.y + h}, n3);
        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;

        if (amin - bmax > 0 || bmin - amax > 0) return false; // separation along n2
        return true; // no axis of separation


    }
    return false;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OBB functions

OBB::OBB() {
    half_height = Vec{0, 0};
    half_width = Vec{0, 0};
    orient = 0;
}

OBB::OBB(Vec hh, Vec hw, Vec centre) {
half_height = hh;
half_width = hw;
position = centre;
double angle = std::atan(hh.x / hh.y);
orient = angle;
if (hh.x < 0 && hh.y > 0) orient = 2*M_PI + angle;
if (hh.x > 0 && hh.y < 0) orient = M_PI / 2 + std::abs(angle);
}

double OBB::get_inertia(double mass){
    return 1.0/12 * mass * (pow(half_height.get_size() * 2, 2) + pow(half_width.get_size() * 2, 2));
}

Rectangle OBB::get_bounding_box() {

    Rectangle rect;
    double max_x = 0;
    double max_y = 0;
    double min_x = 1000000000000;
    double min_y = 10000000000000;
    get_points();
    for (int i = 0; i < 4; i++) {
        double t_max_x = dotProd(points[i], Vec{1, 0});
        double t_max_y = dotProd(points[i], Vec{0, 1});
        double t_min_x = t_max_x;
        double t_min_y = t_max_y;
        if (t_max_x > max_x) max_x = t_max_x;
        if (t_max_y > max_y) max_y = t_max_y;
        if (t_min_x < min_x) min_x = t_min_x;
        if (t_min_y < min_y) min_y = t_min_y;
    }
    rect.max = Vec{max_x, max_y};
    rect.min = Vec{min_x, min_y};
    return rect;

}

bool OBB::intersects(Shape &shape) {
    OBB temp{half_height, half_width, position};
    if (shape.get_type() == Type::AABB) {

        temp.orient = shape.get_orient();
        return shape.intersects(temp);
    }
    if (shape.get_type() == Type::Circle) {
        double angle = -orient;
        Vec og_pos = shape.get_position();
        Vec model_pos = og_pos.rotate(angle);
        Circle circ{shape.get_radius(), model_pos};
        Helper_Rect rect = get_points();
        AABB model_rect{rect.point3.rotate(angle), rect.point1.rotate(angle)};
        return model_rect.intersects(circ);

    }
    if(shape.get_type() == Type::OBB){
        OBB temp_b{shape.get_max(), shape.get_min(), shape.get_position() - position};
        OBB model_this{get_max().rotate(-orient), get_min().rotate(-orient), Vec{}};
 //       model_this.orient = orient;
     //   model_this.rotate(-orient);
        temp_b.orient = (shape.get_orient());
        temp_b.rotate_origin(-orient);
        Helper_Rect rect = model_this.get_points();
        Vec model_max = rect[0];
        Vec model_min = rect[0];
        for(int i = 1; i<4; i++){
            if(rect[i].y > 0 && rect[i].x > 0) model_max = rect[i];
            if(rect[i].y < 0 && rect[i].x < 0) model_min = rect[i];
        }
        AABB model_rect{model_min, model_max};
        return model_rect.intersects(temp_b);
    }
    return false;
}

void OBB::rotate(double angle) {
    half_height = half_height.rotate(angle);
    half_width = half_width.rotate(angle);
    orient += angle;
    if (orient > 2 * M_PI) orient = orient - 2*M_PI;
    if (orient < 0) orient = 2*M_PI + orient;
    get_points();
}

void OBB::rotate_origin(double angle) {
    position = position.rotate(angle);
    half_height = half_height.rotate(angle);
    half_width = half_width.rotate(angle);
    orient += angle;
    if (orient > 2 * M_PI) orient = orient - 2*M_PI;
    if (orient < 0) orient = 2* M_PI + orient;
    get_points();
}

Helper_Rect& OBB::get_points() {
points.point1 = position + half_height + half_width;
points.point2 = position - half_height + half_width;
points.point3 = position - half_height - half_width;
points.point4 = position + half_height - half_width;
return points;

}






Vec& Edge::operator[](int i){
if (i == 0) return point1;
if (i == 1) return point2;
}

Vec Helper_Rect::operator[](int i) const {
    if (i == 0) return point1;
    if (i == 1) return point2;
    if (i == 2) return point3;
    if (i == 3) return point4;
}

bool does_rect_intersect(Rectangle &r1, Rectangle &r2) {
    // check if there is separation along the x-axis
    if (r1.min.x > r2.max.x || r1.max.x < r2.min.x) return false;
    // check if there is separation along the y-axis
    if (r1.min.y > r2.max.y || r1.max.y < r2.min.y) return false;
    return true;
}