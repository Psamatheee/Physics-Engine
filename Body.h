//
// Created by julia on 05/05/22.
//

#ifndef ENGINE_BODY_H
#define ENGINE_BODY_H
#include <cmath>
#include "Geometry.cpp"



class Body{
public:
    Body(Shape& shape, double restitution_const, double m, Vec vect) : shape(shape), rest_const{restitution_const}, mass{m}{
        inv_mass= 1/mass;
        velocity.set_x(vect.get_x());
        velocity.set_y(vect.get_y());

    };

    //getters
    double get_e() const{return rest_const;}
    double get_mass() const{return mass;}
    Vec& get_velocity(){return velocity;}
    Vec get_position(){return shape.get_position();}
    double get_inv_mass() const{return inv_mass;}
    Shape& get_shape() const {return shape;}

    //setters
    void set_velocity(double x, double y){
        velocity.set_x(x);
        velocity.set_y(y);
    }
    void set_position(Vec v){
        shape.set_position(v.get_x(),v.get_y());
    }
    void set_position(double xx, double yy){
        shape.set_position(xx,yy);
    }

    bool collides_wall(double h, double w);
    void integrate(double dt, double w, double h);

    friend bool does_intersect(Body& a, Body& b);


private:
    Shape& shape;
    double rest_const;
    double mass;
    double inv_mass;
    Vec velocity;


};


#endif //ENGINE_BODY_H
