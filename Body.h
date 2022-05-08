//
// Created by julia on 05/05/22.
//

#ifndef ENGINE_BODY_H
#define ENGINE_BODY_H
#include <cmath>
#include "Geometry.cpp"



class Body{
public:
    Body(Shape& shape, float restitution_const, float m, Vec vect) : shape(shape), rest_const{restitution_const}, mass{m}{
        inv_mass= 1/mass;
        velocity.set_x(vect.get_x());
        velocity.set_y(vect.get_y());

    };

    //getters
    float get_e() const{return rest_const;}
    float get_mass() const{return mass;}
    Vec& get_velocity(){return velocity;}
    Vec get_position(){return shape.get_position();}
    float get_inv_mass() const{return inv_mass;}
    Shape& get_shape() const {return shape;}

    //setters
    void set_velocity(float x, float y){
        velocity.set_x(x);
        velocity.set_y(y);
    }
    void set_position(Vec v){
        shape.set_position(v.get_x(),v.get_y());
    }
    void set_position(float xx, float yy){
        shape.set_position(xx,yy);
    }

    bool collides_wall(float h, float w);
    void integrate(float dt, float w, float h);

    friend bool does_intersect(Body& a, Body& b);


private:
    Shape& shape;
    float rest_const;
    float mass;
    float inv_mass;
    Vec velocity;


};


#endif //ENGINE_BODY_H
