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
        gravity = 0;
    }else{
        inv_mass = 1/mass;
        inertia = shape.get_inertia(mass);
        inv_inertia = 1/inertia;
        gravity = 980;
    }

    current = shape.get_position();
    previous = shape.get_position();
    render_pos = shape.get_position();

    angular_vel = 0;
    angle = 0;
    edge.point1 = Vec{};
    edge.point2 = Vec{};

};

void Body::apply_impulse(Vec imp, Vec normal) {
    velocity = velocity + inv_mass * imp;
    double angular_impulse = -cross(normal, imp);
    angular_vel += inv_inertia * angular_impulse;
}

void Body::integrate(double dt) {
    //translation
    Vec dr{dt * velocity.x, dt * velocity.y};
    double new_x = get_position().x + dr.x;
    double new_y = get_position().y + dr.y;
    set_position(new_x, new_y);

    //rotation
    shape.rotate(angular_vel * dt);
    angle = shape.get_orient();

}


