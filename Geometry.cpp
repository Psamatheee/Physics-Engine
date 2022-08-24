//
// Created by julia on 08/05/22.
//

#include <cfloat>
#include "Geometry.h"
#include <iostream>

void Vec::normalize() {
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
    return Vec{v.get_x() * num, v.get_y() * num};
}
bool operator==(Vec v1, Vec v2){
    if(v1.x == v2.x && v1.y == v2.y) return true;
    return false;
}
Vec Vec::orthogonalize() {
    Vec v  =  Vec{y,-x};
    v.normalize();
    return v;
}

//Vector helper functions
double dotProd(const Vec &v1, const Vec &v2) {
    return v1.get_x() * v2.get_x() + v1.get_y() * v2.get_y();
}
double cross(Vec a, Vec b){
    return a.get_x()*b.get_y() - a.get_y()*b.get_x();
}
Vec cross( const Vec& a, double s )
{
    return Vec{s * a.get_y(), -s * a.get_x() };
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Circle Functions

bool Circle::intersects(Shape &shape) {
    if (shape.get_type() == Type::Circle) {

        //Circle circ = shape;
        //shape.get_centre();
        double distance_sqr = pow(centre.get_x() - shape.get_x(), 2) + pow(centre.get_y() - shape.get_y(), 2);
        return pow(radius + shape.get_radius(), 2) >= distance_sqr;
    }
    if (shape.get_type() == Type::AABB) {
        Circle c{radius, centre};
        return shape.intersects(c);
    }
    if (shape.get_type() == Type::OBB) {
        OBB temp{shape.get_max(), shape.get_min(), shape.get_position()};
        temp.orient = shape.get_orient();
        Circle circ{radius, centre};
        return temp.intersects(circ);
    }
    return false;
}


Rectangle Circle::get_bounding_box() {
    Rectangle bb;
    bb.min.set_x(get_position().get_x() - radius);
    bb.min.set_y(get_position().get_y() - radius);
    bb.max.set_x(get_position().get_x() + radius);
    bb.max.set_y(get_position().get_y() + radius);
    return bb;
}

bool does_circle_intersect(Circle &c1, Circle &c2) {
    //using sqr instead of sqrt as it's more efficient
    double distance_sqr = pow(c1.get_x() - c2.get_x(), 2) + pow(c1.get_y() - c2.get_y(), 2);
    return pow(c1.get_radius() + c2.get_radius(), 2) >= distance_sqr;
}

//get how far in the circles are intersecting each other
double get_depth(Shape &a, Shape &b) {
    if (a.get_type() == Type::Circle && b.get_type() == Type::Circle) {
        double distance_sqr = pow(a.get_x() - b.get_x(), 2) +
                              pow(a.get_y() - b.get_y(), 2);
        double distance = sqrt(distance_sqr);
        return a.get_radius() + b.get_radius() - distance;
    }
    return 0;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AABB functions
Vec get_closest_point(Rectangle &r, Circle &c) {
    Vec centre = c.get_position();
    double x = centre.get_x();
    double y = centre.get_y();
    double eeee = 0;
    if (x < r.max.get_x() && x > r.min.get_x() && y > r.min.get_y() && y < r.max.get_y()) {
        double up = r.max.get_y() - y;
        double down = y - r.min.get_y();
        double left = x - r.min.get_x();
        double right = r.max.get_x() - x;
        if (up < down && up < left && up < right) {
            return Vec{x, r.max.get_y()};
        } else if (down < left && down < right && down < up) {
            return Vec{x, r.min.get_y()};
        } else if (left < down && left < right && left < up) {
            return Vec{r.min.get_x(), y};
        } else {
            return Vec{r.max.get_x(), y};
        }
    }
    if (x < r.min.get_x()) {
        if (y > r.max.get_y()) {
            return Vec{r.min.get_x(), r.max.get_y()};
        } else if (y < r.min.get_y()) {
            return Vec{r.min.get_x(), r.min.get_y()};
        } else {
            return Vec{r.min.get_x(), y};
        }
    } else if (x > r.max.get_x()) {
        if (y > r.max.get_y()) {
            return Vec{r.max.get_x(), r.max.get_y()};
        } else if (y < r.min.get_y()) {
            return Vec{r.max.get_x(), r.min.get_y()};
        } else {
            return Vec{r.max.get_x(), y};
        }
    } else {
        if (y > r.max.get_y()) {
            return Vec{x, r.max.get_y()};
        } else {
            return Vec{x, r.min.get_y()};
        }
    }
}

void AABB::set_position(double xx, double yy) {
    double width = max.get_x() - min.get_x();
    double height = max.get_y() - min.get_y();
    //  std::cout<<width << " "<<height<<"\n";
    min.set_x(xx - width);
    min.set_y(yy - height);
    max.set_y(yy);
    max.set_x(xx);
}

bool AABB::intersects(Shape &shape) {
    if (shape.get_type() == Type::AABB) {
        if (min.get_x() > shape.get_max().get_x() || max.get_x() < shape.get_min().get_x()) return false;
        // check if there is separation along the y-.get_x()is
        if (min.get_y() > shape.get_max().get_y() || max.get_y() < shape.get_min().get_y()) return false;

        std::cout << "inter\n";
        return true;
    }
    if (shape.get_type() == Type::Circle) {
        Rectangle r = Rectangle{min, max};
        Vec pos = shape.get_position();
        bool inside = pos.get_x() > min.get_x() && pos.get_x() < max.get_x() && pos.get_y() < max.get_y() && pos.get_y() > min.get_y();
        if(inside) return true;
        Circle c{shape.get_radius(), shape.get_position()};
        Vec closest = get_closest_point(r, c);
        double distance_sqr = pow(closest.get_x() - shape.get_x(), 2) + pow(closest.get_y() - shape.get_y(), 2);
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

        double w = (max.get_x() - min.get_x());
        double h = (max.get_y() - min.get_y());
        //go through all normals to check if there's a separation axis
        //n0
        double amax = max.get_x();
        double amin = min.get_x();

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
        amax = max.get_y();
        amin = min.get_y();
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
        double temp = dotProd(Vec{max.get_x(), max.get_y() - h}, n2);
        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;
        temp = dotProd(min, n2);

        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;
        temp = dotProd(Vec{min.get_x(), min.get_y() + h}, n2);
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
        temp = dotProd(Vec{max.get_x(), max.get_y() - h}, n3);
        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;
        temp = dotProd(min, n3);

        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;
        temp = dotProd(Vec{min.get_x(), min.get_y() + h}, n3);
        if (amax < temp) amax = temp;
        if (temp < amin) amin = temp;

        if (amin - bmax > 0 || bmin - amax > 0) return false; // separation along n2
        return true; // no axis of separation


    }
    return false;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OBB functions

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
        double conv = M_PI / 180;
        double cos = std::cos((angle ));
        double sin = std::sin(angle );
        Matrix m{cos, sin, -sin, cos};
        Vec model_pos = m * og_pos;
        Circle circ{shape.get_radius(), model_pos};
        Helper_Rect rect = get_points();
        AABB model_rect{m * rect.point3, m * rect.point1};
        return model_rect.intersects(circ);

    }
    if(shape.get_type() == Type::OBB){
        OBB temp_b{shape.get_max(), shape.get_min(), shape.get_position() - position};
        OBB model_this{get_max(), get_min(), Vec{}};
        model_this.orient = orient;
        model_this.rotate(-orient);
        temp_b.orient = (shape.get_orient());
        temp_b.rotate_origin(-orient);

        double conv = M_PI / 180;
        double angle = -orient;
        double cos = std::cos((angle));
        double sin = std::sin(angle );
        Matrix m{cos, sin, -sin, cos};
        Helper_Rect rect = model_this.get_points();
        Vec model_max = rect[0];
        Vec model_min = rect[0];
        for(int i = 1; i<4; i++){
            if(rect[i].get_x() > model_max.get_y() && rect[i].get_x() > model_max.get_x()) model_max = rect[i];
            if(rect[i].get_x() < model_min.get_y() && rect[i].get_x() < model_min.get_x()) model_min = rect[i];
        }
        AABB model_rect{model_min, model_max};
        return model_rect.intersects(temp_b);
    }
    return false;
}

void OBB::rotate(double angle) {
    double conv = M_PI / 180;
    double cos = std::cos((angle ));
    double sin = std::sin(angle );
    Matrix m{cos, sin, -sin, cos};
    half_height = m * half_height;
    half_width = m * half_width;
    orient += angle;
    if (orient > 2 * M_PI) orient = orient - 2*M_PI;
    if (orient < 0) orient = 2*M_PI + orient;
    get_points();
}

void OBB::rotate_origin(double angle) {
    double conv = M_PI / 180;
    double cos = std::cos((angle ));
    double sin = std::sin(angle );
    Matrix m{cos, sin, -sin, cos};
    position = (m * position);
    half_height = m * half_height;
    half_width = m * half_width;
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



double get_min_point(Helper_Rect& rect, Vec& axis, Vec& point){
    double pen = FLT_MAX;
    point = Vec{};
    for(int i =0; i < 4; i++){
        if(dotProd(axis,rect[i]) < pen) {
            pen =  dotProd(axis,rect[i]);
            point = rect[i];
        }
    }
    return pen;
}
double get_max_point(Helper_Rect& rect, Vec& axis, Vec& point){
    double pen = 0;
    point = Vec{};
    for(int i =0; i < 4; i++){
        if(dotProd(axis,rect[i]) > pen) {
            pen =  dotProd(axis,rect[i]);
            point = rect[i];
        }
    }
    return pen;
}

double get_collision_normal(OBB& a, OBB& b, Edge& edge, int& edge_num){
    Helper_Rect b_rect = b.get_points();
    Helper_Rect a_rect = a.get_points();

    double penetration = FLT_MAX;

    Vec n1 = a.get_max();
    Vec n2 = a.get_min();
    n1.normalize();
    n2.normalize();

    Vec max_a = a.get_position() + a.half_height + a.half_width;
    Vec min_a = a.get_position() - a.half_height - a.half_width;

    //n1
    Vec max_b{};
    Vec min_b{};
    get_max_point(b_rect, n1, max_b);
    get_min_point(b_rect, n1, min_b);
    double aa = dotProd(max_a, n1) - dotProd(min_b, n1);
    double bb = dotProd(max_b, n1) - dotProd(min_a, n1);

    if(aa < bb && aa < penetration){
        penetration = aa;
        edge[0] = a_rect[3];
        edge[1] = a_rect[0];
        edge_num = 4;
    }
    if(bb < aa && bb < penetration){
        penetration = bb;
        edge[0] = a_rect[1];
        edge[1] = a_rect[2];
        edge_num = 2;
    }

    //n2
    get_max_point(b_rect, n2, max_b);
    get_min_point(b_rect, n2, min_b);
     aa = dotProd(max_a, n2) - dotProd(min_b, n2);
     bb = dotProd(max_b, n2) - dotProd(min_a, n2);

    if(aa < bb && aa < penetration){
        penetration = aa;
        edge[0] = a_rect[0];
        edge[1] = a_rect[1];
        edge_num = 1;
    }
    if(bb < aa && bb < penetration){
        penetration = bb;
        edge[0] = a_rect[2];
        edge[1] = a_rect[3];
        edge_num = 3;
    }

    return penetration;

}

void set_incident_edge(OBB& box, Edge& edge, Vec& normal, int& edge_num){
    Helper_Rect rect = box.get_points();
    double d = 1;
    for(int i = 0; i < 4; i++){
        Vec edge_normal = i != 3? rect[i] - rect[i+1] : rect[i] - rect[0];
        edge_normal = edge_normal.orthogonalize();
        double dot = dotProd(edge_normal, normal);
        if( dot <= d) {
            d = dot;
            edge.point1 = rect[i];
            edge.point2 = i != 3? rect[i+1] : rect[0];
            edge_num = i+1;
        }
    }
}