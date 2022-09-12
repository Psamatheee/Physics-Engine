//
// Created by julia on 08/05/22.
//

#ifndef ENGINE_GEOMETRY_H
#define ENGINE_GEOMETRY_H

#include <cmath>
#include <vector>

enum class Type {
    Circle, AABB, OBB
};

class Vec {
public:
    Vec(double xx, double yy) : x{xx}, y{yy} {size = sqrt(x * x + y * y); }
    Vec() : x{0}, y{0} { size = 0; }

    double get_size() const { return sqrt(x * x + y * y); }
    void normalize();
    Vec orthogonalize();
    Vec rotate(double angle); //(clockwise)

    //operators
    friend Vec operator-(const Vec &v1, const Vec &v2);
    friend Vec operator+(const Vec &v1, const Vec &v2);
    friend Vec operator*(double num, const Vec &v);
    friend bool operator==(const Vec v1, const Vec v2);

    double x;
    double y;
    double size;
};

struct Edge {
    Vec point1;
    Vec point2;

    Vec &operator[](int i) {
        if (i == 0) return point1;
        if (i == 1) return point2;
    }
};

struct Matrix {
    struct {
        double m00, m01;
        double m10, m11;
    };

    //gets a matrix that will rotate a vector by the provided angle in radians CLOCKWISE


    Vec operator*(Vec a) const {
        return Vec{m00 * a.x + m01 * a.y, m10 * a.x + m11 * a.y};
    }
    Vec operator*(Vec* a) {
        return Vec{(m00 * a->x + m01 * a->y) , (m10 * a->x + m11 * a->y)};
    }

};

Matrix get_rotation_m(double angle);

struct Rectangle {
    Vec min;
    Vec max;
};

//represents 4 points of a rectangle starting from top right corner and going clockwise;
struct Helper_Rect {
    Vec operator[](int i) const {
        if (i == 0) return point1;
        if (i == 1) return point2;
        if (i == 2) return point3;
        if (i == 3) return point4;
    }

    Vec point1;
    Vec point2;
    Vec point3;
    Vec point4;
};

bool does_rect_intersect(Rectangle &r1, Rectangle &r2) {
    // check if there is separation along the x-axis
    if (r1.min.x > r2.max.x || r1.max.x < r2.min.x) return false;
    // check if there is separation along the y-axis
    if (r1.min.y > r2.max.y || r1.max.y < r2.min.y) return false;
    return true;
}

class Shape {
public:

    //getters & setters
    virtual Vec get_position() = 0;
    virtual double get_x() = 0;
    virtual double get_y() = 0;
    virtual Type get_type() = 0;
    virtual void set_position(double xx, double yy) = 0;
    virtual Rectangle get_bounding_box() = 0;

    //general
    virtual bool intersects(Shape &shape) = 0;
    virtual void rotate(double angle) = 0;
    virtual double get_orient() = 0; //clockwise

    //Circle functions
    virtual double get_radius() = 0;

    //AABB functions
    virtual Vec get_min() = 0;
    virtual Vec get_max() = 0;

    //OBB functions
    virtual Helper_Rect &get_points() = 0;
};

/* Oriented Bounding Box
 *
 * Used to represent a rotated rectangle/square.
 * half height is the vector that represents the distance from the origin of the rectangle to its edge along the y-axis
 * when it's oriented at 0 degrees, the get_max() function also returns this vector even though this is
 * an AABB function for simplicity
 * half width is the same but with the x-axis and get_min() function
 *
 * the orient angle represents the amount the box is rotated from the y-axis CLOCKWISE. The angle is in radians
 *
 */

class OBB : public Shape {
public:
    OBB() {
        half_height = Vec{0, 0};
        half_width = Vec{0, 0};
        orient = 0;
    }

    OBB(Vec hh, Vec hw, Vec centre) {
        half_height = hh;
        half_width = hw;
        position = centre;
        double angle = std::atan(hh.x / hh.y);
        orient = angle;
        if (hh.x < 0 && hh.y > 0) orient = 2*M_PI + angle;
        if (hh.x > 0 && hh.y < 0) orient = M_PI / 2 + std::abs(angle);
    }

    //getters
    Vec get_position() override { return position; }
    double get_x() override { return position.x; }
    double get_y() override { return position.y; }
    double get_orient() override { return orient; }
    Type get_type() override { return Type::OBB; }

    //setters
    void set_position(double xx, double yy) override { position = Vec{xx, yy}; }

    Rectangle get_bounding_box() override;
    bool intersects(Shape &shape) override;
    void rotate(double angle) override;
    Helper_Rect &get_points() override;
    void rotate_origin(double angle);

    //obb functions

    //throwaway functions
    double get_radius() override { return 0; }
    Vec get_min() override { return half_width; }
    Vec get_max() override { return half_height; }

    Vec position;
    Helper_Rect points;
    Vec half_height;
    Vec half_width;
    double orient;
};

class Circle : public Shape {
public:
    Circle(double r, Vec c) : radius{r}, centre{c} { orient = 0; }

    Circle() : radius{50}, centre(100, 100) {}

    //getters
    double get_radius() override { return radius; }

    Vec &get_centre() { return centre; }

    Vec get_position() override { return centre; }

    double get_x() override { return centre.x; }

    double get_y() override { return centre.y; }

    Type get_type() override { return Type::Circle; }

    double get_orient() override { return orient; }

    //setters
    void set_position(double xx, double yy) override {
        centre.x = xx;
        centre.y = yy;
    }

    void set_orient(double angle) { orient = angle; }

    void rotate(double angle) override {
        orient += angle;
        if (orient > 2 * M_PI) orient = orient - 2 * M_PI;
        if (orient < 0) orient = 2 * M_PI + orient;
    };

    Rectangle get_bounding_box() override;

    bool intersects(Shape &shape) override;

    //throwaway shape functions
    Vec get_max() override { return Vec{}; }

    Vec get_min() override { return Vec{}; }

    Helper_Rect &get_points() override {};

private:
    const double radius;
    Vec centre; //position vector to the centre of the circle
    double orient;
};

class AABB : public Shape {
public:
    AABB(Vec min, Vec max) : min(min), max(max) {};

    //getters
    Vec get_position() override { return max; }

    double get_x() override { return max.x; }

    double get_y() override { return max.y; }

    Type get_type() override { return Type::AABB; }

    Vec get_min() override { return min; }

    Vec get_max() override { return max; }

    //setters
    void set_position(double xx, double yy) override;

    Rectangle get_bounding_box() override { return Rectangle{min, max}; }

    bool intersects(Shape &shape) override;

    //throwaway shape functions
    double get_orient() override { return 0; }

    double get_radius() override { return 0; }

    void rotate(double angle) override {}

    Helper_Rect &get_points() override {};

private:
    Vec min;
    Vec max;

};


#endif //ENGINE_GEOMETRY_H
