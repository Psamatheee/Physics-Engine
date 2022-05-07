//
// Created by julia on 05/05/22.
//

#ifndef ENGINE_BODY_H
#define ENGINE_BODY_H
#include <cmath>

class Vect{
public:
    Vect(float xx, float yy) : x{xx}, y{yy}{
        size = sqrt(x*x + y*y);
    }
    Vect() : x{0}, y{0}{
        size = 0;
    }

    float get_x(){return x;}
    float get_y(){return y;}

    void set_x(float xx){x = xx; }
    void set_y(float yy){y = yy;}

    void normalize();

    friend Vect operator-(Vect& v1, Vect& v2);
    friend Vect operator+(Vect& v1, Vect& v2);
    friend Vect operator*(float num, Vect& v);

private:
    float x;
    float y;
    float size;
};

class Circle{
public:
    Circle(float r, Vect c) : radius{r}, centre{c}{}
    float get_radius() const{return radius;}
    Vect& get_centre(){return centre;}
    void set_position(float xx, float yy){
        centre.set_x(xx);
        centre.set_y(yy);
    }



private:
    const float radius;
    Vect centre; //position vector to the centre of the circle
};

class Body{
public:
    Body(Circle circ, float restitution_const, float m, Vect vect) : circle{circ}, rest_const{restitution_const}, mass{m}{
        inv_mass= 1/mass;
        velocity.set_x(vect.get_x());
        velocity.set_y(vect.get_y());

    };

    //getters
    float get_e(){return rest_const;}
    float get_mass(){return mass;}
    Vect& get_velocity(){return velocity;}
    Vect& get_position(){return circle.get_centre();}
    Circle get_circle(){return circle;}
    float get_inv_mass(){return inv_mass;}

    //setters
    void set_velocity(float x, float y){
        velocity.set_x(x);
        velocity.set_y(y);
    }
    void set_position(Vect v){
        circle.set_position(v.get_x(),v.get_y());
    }
    void set_position(float xx, float yy){
        circle.set_position(xx,yy);
    }

    float get_radius(){return circle.get_radius();}
    bool collides_wall(float h, float w);
    void increase_velocity(Vect v){
        velocity = velocity + v;
    }
    void integrate(float dt, float w, float h);

    friend bool does_intersect(Body& a, Body& b);


private:
    Circle circle;
    float rest_const;
    float mass;
    float inv_mass;
    Vect velocity;


};


#endif //ENGINE_BODY_H
