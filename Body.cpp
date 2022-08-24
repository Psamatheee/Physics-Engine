//
// Created by julia on 05/05/22.
//

#include "Body.h"
#include <algorithm>

void Body::apply_impulse(Vec imp, Vec normal) {
    velocity = velocity + inv_mass * imp;
    double angular_impulse = -cross(normal, imp);
    angular_vel += inv_inertia * angular_impulse;
}

void Body::integrate(double dt, double w, double h) {
    Vec dr{dt * velocity.get_x(), dt * velocity.get_y()};
    double new_x = get_position().get_x() + dr.get_x();
    double new_y = get_position().get_y() + dr.get_y();
    set_position(new_x, new_y);
    shape.rotate(angular_vel * dt);
    angle = shape.get_orient();
}

struct Pair {
    Body *a;
    Body *b;
};

bool operator==(Body a, Body b) {
    bool vecs = (a.get_position().get_x() == b.get_position().get_x() &&
                 a.get_position().get_y() == b.get_position().get_y() &&
                 a.get_velocity().get_y() == b.get_velocity().get_y() &&
                 a.get_velocity().get_x() == b.get_velocity().get_x());
    bool mass = (a.get_mass() == b.get_mass());
    return mass && vecs;
}

bool operator!=(Body a, Body b) {
    return !(a == b);
}