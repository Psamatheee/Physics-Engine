//
// Created by julia on 05/05/22.
//

#include "Body.h"
#include <algorithm>

Body::Body(Shape& shape, double m) : shape(shape), mass{m}{
    if(mass == 0){
        inv_mass = 0;
        inertia = 0;
        inv_inertia = 0;
    }else{
        inv_mass = 1/mass;
        inertia = shape.get_inertia(mass);
        inv_inertia = 1/inertia;
    }

    current = shape.get_position();
    previous = shape.get_position();
    render_pos = shape.get_position();
    if(mass == 0) {

        gravity = 0;
    }else{
        gravity = 980;
    }
    angular_vel = 0;
    angle = 0;
    edge.point1 = Vec{};
    edge.point2 = Vec{};
    rest_const = 0.75;

};

void Body::apply_impulse(Vec imp, Vec normal) {
    velocity = velocity + inv_mass * imp;
    double angular_impulse = -cross(normal, imp);
    angular_vel += inv_inertia * angular_impulse;
}

void Body::integrate(double dt) {
    Vec dr{dt * velocity.x, dt * velocity.y};
    double new_x = get_position().x + dr.x;
    double new_y = get_position().y + dr.y;
    set_position(new_x, new_y);
    shape.rotate(angular_vel * dt);
    angle = shape.get_orient();
}



bool operator==(Body a, Body b) {
    bool vecs = (a.get_position().x == b.get_position().x &&
                 a.get_position().y == b.get_position().y &&
                 a.get_velocity().y == b.get_velocity().y &&
                 a.get_velocity().x == b.get_velocity().x);
    bool mass = (a.get_mass() == b.get_mass());
    return mass && vecs;
}

bool operator!=(Body a, Body b) {
    return !(a == b);
}