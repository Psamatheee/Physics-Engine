//
// Created by julia on 08/05/22.
//

#ifndef ENGINE_GEOMETRY_H
#define ENGINE_GEOMETRY_H
#include <cmath>

enum class Type {Base, Circle, AABB, OBB};

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
        Vec orthogonalize();//orthogonal

        friend Vec operator-(const Vec& v1, const Vec& v2);
        friend Vec operator+(const Vec& v1,const  Vec& v2);
        friend Vec operator*(double num, const Vec& v);
        friend bool operator==(const Vec v1, const Vec v2);

        private:
        double x;
        double y;
        double size;
};

struct Edge{
    Vec point1;
    Vec point2;


    Vec& operator[] (int i){
        if(i == 0) return point1;
        if(i == 1) return point2;
    }
};



struct Matrix
{
        struct
        {
            double m00, m01;
            double m10, m11;
        };

    Vec operator*(Vec a) const{
        return Vec{m00*a.get_x() + m01*a.get_y() , m10*a.get_x() + m11*a.get_y()};
    }

};


struct Rectangle{
    Vec min;
    Vec max;
};

//represents 4 points of a rectangle starting from top right corner and going clockwise;
struct Helper_Rect{
    Vec operator[](int i) const{
        if(i == 0) return point1;
        if(i == 1) return point2;
        if(i == 2) return point3;
        if(i == 3) return point4;

    }
    Vec point1;
    Vec point2;
    Vec point3;
    Vec point4;
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
  //  ~Shape();
    virtual Vec get_position() = 0;
    virtual double get_x() = 0;
    virtual double get_y() = 0;
    virtual Type get_type() = 0;

    virtual void set_position(double xx, double yy) = 0;
    virtual Rectangle get_bounding_box() = 0;

    virtual bool intersects(Shape& shape) = 0;
    virtual void rotate(double angle) =0;
    virtual double get_orient() =0;

    //Circle functions
    virtual double get_radius() = 0;

    //AABB functions
    virtual Vec get_min() =0;
    virtual Vec get_max() =0;

    //OBB functions
    virtual Helper_Rect& get_points() =0;
};

class OBB : public Shape{
public:
    OBB(){
        h = 0;
        w = 0;
        half_height = Vec{0,0};
        half_width = Vec{0,0};
        orient = 0;
    }
    OBB(Vec hh, Vec hw, Vec centre){
        half_height = hh;
        half_width = hw;
        position = centre;
        double angle = std::atan(hh.get_x()/hh.get_y());
        orient = angle;
        if(hh.get_x() < 0 && hh.get_y() > 0 ) orient = 360 + angle;
        if(hh.get_x() > 0 && hh.get_y() < 0 ) orient = 90 + std::abs(angle);

    }

    //getters
    Vec get_position() override{return position;}
    double get_x() override{return position.get_x();}
    double get_y() override{return position.get_y();}
    double get_orient()override{return orient;}
    Type get_type() override{return Type::OBB;}
    std::vector<Edge> get_edges();
    //setters
    void set_position (double xx, double yy) override{position = Vec{xx,yy};}

    Rectangle get_bounding_box() override;
    bool intersects(Shape& shape) override;
    void rotate(double angle) override;
    Helper_Rect& get_points() override;

    void rotate_origin(double angle);

    //obb functions
    Vec get_normal(int i);

    //throwaway functions
     double get_radius() override {return 0;}
     Vec get_min() override {return half_width;}
     Vec get_max() override {return half_height ;}


    Vec position;
    Helper_Rect points;
    Vec half_height;
    Vec half_width;
    double h;
    double w;
    double orient;
};

class Circle : public Shape{
public:
    Circle(double r, Vec c) : radius{r}, centre{c}{}
    Circle() : radius{50}, centre(100,100){}

    //getters
    double get_radius() override{return radius;}
    Vec& get_centre(){return centre;}
    Vec get_position() override{return centre;}
    double get_x() override{return centre.get_x();}
    double get_y() override{return centre.get_y();}
    Type get_type() override{return Type::Circle;}
    double get_orient() override {return 0;}
    //setters
    void set_position(double xx, double yy) override{
        centre.set_x(xx);
        centre.set_y(yy);
    }

    void rotate(double angle) override{}
    Rectangle get_bounding_box() override;
    bool intersects(Shape& shape) override;

    //throwaway shape functions
    Vec get_max() override{return Vec{};}
    Vec get_min() override{return Vec{};}
    Helper_Rect& get_points() override{ };

private:
    const double radius;
    Vec centre; //position vector to the centre of the circle
};

class AABB : public Shape{
public:
    AABB(Vec min, Vec max) : min(min), max(max){};

    //getters
    Vec get_position() override {return max;}
    double get_x() override {return max.get_x();}
    double get_y() override {return max.get_y();}
    Type get_type() override {return Type::AABB;}
    Vec get_min() override {return min;}
    Vec get_max() override {return max;}
    //setters
    void set_position(double xx, double yy)override;

    Rectangle get_bounding_box() override {return Rectangle{min,max};}
    bool intersects(Shape& shape) override;

    //throwaway shape functions
    double get_orient() override {return 0;}
    double get_radius() override{return 0;}
    void rotate(double angle) override{}
    Helper_Rect& get_points() override{ };

private:
    Vec min;
    Vec max;

};


#endif //ENGINE_GEOMETRY_H
