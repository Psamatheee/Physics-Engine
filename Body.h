//
// Created by julia on 05/05/22.
//

#ifndef ENGINE_BODY_H
#define ENGINE_BODY_H
#include <cmath>
#include "Geometry.h"



struct Body{
public:
    explicit Body(std::unique_ptr<Shape> shape);

    void apply_impulse(Vec impulse, Vec normal_vec);

    friend bool is_intersecting(Body& a, Body& b);

    void set_static();
    Vec get_position() const{
        return shape->get_position();
    }
    void set_position(Vec v){
        shape->set_position(v.x,v.y);
    }
    void set_position(double xx, double yy){
        shape->set_position(xx,yy);
    }

    void integrate(double dt);
    double gravity;
    double mass;
    double inv_mass;
    double dynamic_coeff = 0.28;
    double angular_vel;
    double angle;
    double inertia;
    double inv_inertia;
    std::vector<Vec> contacts;
    std::unique_ptr<Shape> shape;
    Vec velocity;
    Vec current;
    Vec previous;
    Vec render_position;
};




#endif //ENGINE_BODY_H
