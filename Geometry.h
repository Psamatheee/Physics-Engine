//
// Created by julia on 08/05/22.
//

#ifndef ENGINE_GEOMETRY_H
#define ENGINE_GEOMETRY_H
#include <cmath>
enum class Type {Base, Circle, AABB, OBB};
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

            double cross(Vec a, Vec b){
            return a.get_x()*b.get_y() - a.get_y()*b.get_y();
        }
        void normalize();

        friend Vec operator-(const Vec& v1, const Vec& v2);
        friend Vec operator+(const Vec& v1,const  Vec& v2);
        friend Vec operator*(double num, const Vec& v);

        double angle_from_xaxis();

        private:
        double x;
        double y;
        double size;
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
  //  virtual Boundary collides_boundary(double w, double h) = 0;

    //Circle functions
    virtual double get_radius() = 0;

    //AABB functions
    virtual Vec get_min() =0;
    virtual Vec get_max() =0;

    //OBB functions
    virtual void rotate(double angle) =0;
    virtual Helper_Rect& get_points() =0;

    virtual double get_orient() =0;

};

class OBB : public Shape{
public:
    OBB(){
        h = 0;
        w = 0;
        half_height = Vec{0,0};
        half_width = Vec{0,0};
    }
    OBB(double height, double width, Vec centre){
        h = height;
        w = width;
        half_height = Vec{0,height/2};
        half_width = Vec{width/2,0};
        points.point1 = Vec{};
        position = centre;
    }
double get_orient()override{return orient;}
    //Shape functions
    Vec get_position() override{
        return position;
    }
    double get_x() override{
        return position.get_x();
    }
    double get_y() override{
        return  position.get_y();
    }
    Type get_type() override{
        return Type::OBB;
    }

    void set_position (double xx, double yy) override{
        position = Vec{xx,yy};
    }
    Rectangle get_bounding_box() override;

    bool intersects(Shape& shape) override{
        if(shape.get_type() == Type::AABB){
            OBB temp{half_height.get_size(),half_width.get_size(),position};
            return shape.intersects(temp);
        }
        return false;
    }
    //Boundary collides_boundary(double w, double h) override ;

    //clockwise
    void rotate(double angle) override{
        double conv = M_PI / 180;
        double cos = std::cos((angle * conv));
        double sin = std::sin(angle * conv);
        Matrix m{cos, sin, -sin, cos};
        half_height = m * half_height;
        half_width = m * half_width;
        orient += angle;
        if(orient > 360) orient = orient - 360;
        if(orient < 0) orient = 360 + orient;
        get_points();
    }

    Helper_Rect& get_points() override {
        points.point1 = position + half_height + half_width;
        points.point2 = position - half_height + half_width;
        points.point3 = position - half_height - half_width;
        points.point4 = position + half_height - half_width;
        return points;

    }

    //throwaway functions
     double get_radius() override {return 0;}
     Vec get_min() override {return half_width;}
     Vec get_max() override {return half_height;}


    Vec position;
    Helper_Rect points;
    Vec half_height;
    Vec half_width;
    double h;
    double w;
    double orient = 0;
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
    void rotate(double angle) override{}
    Helper_Rect& get_points() override{ };

    Rectangle get_bounding_box() override;
double get_orient() override {return 0;}
    void set_position(double xx, double yy) override{
        centre.set_x(xx);
        centre.set_y(yy);
    }

   // Vec Get

    bool intersects(Shape& shape) override;
   Boundary collides_boundary(double w, double h) ;

private:
    const double radius;
    Vec centre; //position vector to the centre of the circle
};

class AABB : public Shape{
public:
    AABB(Vec min, Vec max) : min(min), max(max){};

    double get_orient() override {return 0;}
    Vec get_position() override {return max;}
    double get_x() override {return max.get_x();}
    double get_y() override {return max.get_y();}
    Type get_type() override {return Type::AABB;}

    Rectangle get_bounding_box() override {return Rectangle{min,max};}

    Vec get_min() override {return min;}
    Vec get_max() override {return max;}
    double get_radius() override{return 0;}
    void rotate(double angle) override{}
    Helper_Rect& get_points() override{ };
    void set_position(double xx, double yy)override;

    bool intersects(Shape& shape) override;
    Boundary collides_boundary(double w, double h) ;

private:
    Vec min;
    Vec max;

};


#endif //ENGINE_GEOMETRY_H
