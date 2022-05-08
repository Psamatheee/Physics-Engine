//
// Created by julia on 08/05/22.
//

#ifndef ENGINE_GEOMETRY_H
#define ENGINE_GEOMETRY_H
#include <cmath>
enum class Type {Base, Circle};
enum class Boundary {Top, Right, Bottom, Left, None};

class Vec{
        public:
        Vec(float xx, float yy) : x{xx}, y{yy}{
            size = sqrtf(x*x + y*y);
        }
        Vec() : x{0}, y{0}{
            size = 0;
        }

        float get_x() const{return x;}
        float get_y() const{return y;}

        void set_x(float xx){x = xx;
            size = sqrtf(x*x + y*y);}
        void set_y(float yy){y = yy;
            size = sqrtf(x*x + y*y);}

        void normalize();

        friend Vec operator-(Vec& v1, Vec& v2);
        friend Vec operator+(Vec& v1, Vec& v2);
        friend Vec operator*(float num, Vec& v);

        private:
        float x;
        float y;
        float size;
};


class Shape {
public:
    virtual Vec get_position() = 0;
    virtual float get_x() = 0;
    virtual float get_y() = 0;
    virtual Type get_type() = 0;

    virtual void set_position(float xx, float yy) = 0;

    virtual bool intersects(Shape& shape) = 0;
    virtual Boundary collides_boundary(float w, float h) = 0;

    //Circle functions
    virtual float get_radius(){return radius;}
private:
    Vec position;
    float radius;
};

class Circle : public Shape{
public:
    Circle(float r, Vec c) : radius{r}, centre{c}{}
    Circle() : radius{50}, centre(100,100){}

    float get_radius() override {return radius;}
    Vec& get_centre(){return centre;}
    Vec get_position() override{return centre;}
    float get_x() override{return centre.get_x();}
    float get_y() override{return centre.get_y();}
    Type get_type() override{return Type::Circle;}

    void set_position(float xx, float yy) override{
        centre.set_x(xx);
        centre.set_y(yy);
    }

    bool intersects(Shape& shape) override;
    Boundary collides_boundary(float w, float h) override;

private:
    const float radius;
    Vec centre; //position vector to the centre of the circle
};


#endif //ENGINE_GEOMETRY_H
