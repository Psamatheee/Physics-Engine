//
// Created by julia on 08/05/22.
//

#ifndef ENGINE_GEOMETRY_H
#define ENGINE_GEOMETRY_H
#include <cmath>
enum class Type {Base, Circle, AABB};
enum class Boundary {Top, Right, Bottom, Left, TR, TL, BR, BL, None};

class Vec{
        public:
        Vec(double xx, double yy) : x{xx}, y{yy}{
            size = sqrt(x*x + y*y);
        }
        Vec() : x{0}, y{0}{
            size = 0;
        }

        double get_x() const{return x;}
        double get_y() const{return y;}
    double get_size() const{return  size;}

        void set_x(double xx){x = xx;
            size = sqrt(x*x + y*y);}
        void set_y(double yy){y = yy;
            size = sqrt(x*x + y*y);}

        void normalize();

        friend Vec operator-(Vec& v1, Vec& v2);
        friend Vec operator+(Vec& v1, Vec& v2);
        friend Vec operator*(double num, Vec& v);

        double angle_from_xaxis();

        private:
        double x;
        double y;
        double size;
};


struct Rectangle{
    Vec min;
    Vec max;
};

bool does_rect_intersect(Rectangle& r1, Rectangle& r2){
    // check if there is separation along the x-axis
    if (r1.min.get_x() > r2.max.get_x() || r1.max.get_x() < r2.min.get_x()) return false;
    // check if there is separation along the y-.get_x()is
    if (r1.min.get_y() > r2.max.get_y()|| r1.max.get_y() < r2.min.get_y()) return false;
    return true;
}


class Shape {
public:
    virtual Vec get_position() = 0;
    virtual double get_x() = 0;
    virtual double get_y() = 0;
    virtual Type get_type() = 0;

    virtual void set_position(double xx, double yy) = 0;
    virtual Rectangle get_bounding_box() = 0;

    virtual bool intersects(Shape& shape) = 0;
    virtual Boundary collides_boundary(double w, double h) = 0;

    //Circle functions
    virtual double get_radius() = 0;

    //AABB functions
    virtual Vec get_min() =0;
    virtual Vec get_max() =0;

};

class Circle : public Shape{
public:
    Circle(double r, Vec c) : radius{r}, centre{c}{}
    Circle() : radius{50}, centre(100,100){}

    double get_radius() override{return radius;}
    Vec& get_centre(){return centre;}
    Vec get_position() override{return centre;}
    double get_x() override{return centre.get_x();}
    double get_y() override{return centre.get_y();}
    Type get_type() override{return Type::Circle;}
    Vec get_max() override{return Vec{};}
    Vec get_min() override{return Vec{};}

    Rectangle get_bounding_box() override;

    void set_position(double xx, double yy) override{
        centre.set_x(xx);
        centre.set_y(yy);
    }

   // Vec Get

    bool intersects(Shape& shape) override;
    Boundary collides_boundary(double w, double h) override;

private:
    const double radius;
    Vec centre; //position vector to the centre of the circle
};

class AABB : public Shape{
public:
    AABB(Vec min, Vec max) : min(min), max(max){};

    Vec get_position() override {return max;}
    double get_x() override {return max.get_x();}
    double get_y() override {return max.get_y();}
    Type get_type() override {return Type::AABB;}

    Rectangle get_bounding_box() override {return Rectangle{min,max};}

    Vec get_min() override {return min;}
    Vec get_max() override {return max;}
    double get_radius() override{return 0;}
    void set_position(double xx, double yy)override;

    bool intersects(Shape& shape) override;
    Boundary collides_boundary(double w, double h) override;

private:
    Vec min;
    Vec max;

};


#endif //ENGINE_GEOMETRY_H
