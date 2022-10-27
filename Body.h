//
// Created by julia on 05/05/22.
//

#ifndef ENGINE_BODY_H
#define ENGINE_BODY_H
#include <cmath>
#include "Geometry.h"



class Body{
public:
    Body(Shape& shape, double m);

    void apply_impulse(Vec impulse, Vec normal_vec);

    //getters
    double get_mass() const{return mass;}
    Vec& get_velocity(){return velocity;}
    Vec get_position()const {return shape.get_position();}
    double get_inv_mass() const{return inv_mass;}
    Shape& get_shape() const {return shape;}
    Vec get_curr(){return current;}
    Vec get_prev(){return previous;}
    Vec get_render()const {return render_pos;}

    //setters
    void set_velocity(double x, double y){
        velocity.x = x;
        velocity.y = y;
    }
    void set_velocity(Vec vel){
        velocity.x = vel.x;
        velocity.y = vel.y;
    }
    void set_position(Vec v){
        shape.set_position(v.x,v.y);
    }
    void set_position(double xx, double yy){
        shape.set_position(xx,yy);
    }
    void set_current(Vec vec){
        current = vec;
    }
    void set_previous(Vec vec){
        previous = vec;
    }
    void set_render(Vec vec){
        render_pos = vec;
    }

    void integrate(double dt);

    double  gravity;
    double mass;
    double inv_mass;
    double dynamic_coeff = 0.28;
    double angular_vel;
    double angle;
    double inertia;
    double inv_inertia;
    Edge edge;
    std::vector<Vec> contacts;

private:
    Shape& shape;
    double rest_const;

    Vec velocity;
    Vec current;
    Vec previous;
    Vec render_pos;
};

struct Pair {
    Body *a;
    Body *b;
};


#endif //ENGINE_BODY_H
