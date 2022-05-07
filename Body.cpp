//
// Created by julia on 05/05/22.
//

#include "Body.h"
#include <algorithm>
//======================================================================================================================
//Vector functions
void Vect::normalize() {
    x = x/size;
    y = y/size;
}

Vect operator-(Vect& v1, Vect& v2){
Vect v{v1.x - v2.x, v1.y - v2.y};
return v;
}

Vect operator+(Vect& v1, Vect& v2){
    Vect v{v1.x + v2.x, v1.y + v2.y};
    return v;
}

Vect operator*(float num, Vect &v) {
    return Vect{v.get_x()* num, v.get_y()*num};
}

//Vector helper functions
float dotProd(Vect& v1, Vect& v2){
    return v1.get_x() * v2.get_x() + v1.get_y() * v2.get_y();
}

Vect scalar_mult(float num, Vect& v){
    return {v.get_x()*num, v.get_y()*num};
}

//====================================================================================================================
//Circle Functions


void Body::integrate(float dt, float w, float h) {
    Vect dr{dt*velocity.get_x(), dt*velocity.get_y()};
    float new_x = get_position().get_x() + dr.get_x();
    float new_y = get_position().get_y() + dr.get_y();
    set_position(new_x , new_y);
    collides_wall(h,w);


}

bool does_circle_intersect(Circle& c1, Circle& c2){
    //using sqr instead of sqrt as it's more efficient
    float distance_sqr = pow(c1.get_centre().get_x() - c2.get_centre().get_x(),2) + pow(c1.get_centre().get_y() - c2.get_centre().get_y(),2);
    return pow(c1.get_radius() + c2.get_radius(),2) >= distance_sqr;
}

float get_depth(Circle c1, Circle c2){
    float distance_sqr = pow(c1.get_centre().get_x() - c2.get_centre().get_x(),2) + pow(c1.get_centre().get_y() - c2.get_centre().get_y(),2);
    float distance = sqrt(distance_sqr);
    return c1.get_radius() + c2.get_radius() - distance;
}

bool does_intersect(Body& a, Body& b){
    return does_circle_intersect(a.circle, b.circle);
};

bool Body::collides_wall(float h, float w){

    if(get_position().get_y() + get_radius() >= h || get_position().get_y() - get_radius() <= 0  ) {
        set_velocity(get_velocity().get_x(), get_velocity().get_y() * -1);
        return true;
    }

    if(get_position().get_x() + get_radius() >= w || get_position().get_x() - get_radius() <= 0  ) {
        set_velocity(get_velocity().get_x() * -1, get_velocity().get_y() );
       return true;
    }

    return false;
}

struct Manifold{
    Body& a;
    Body& b;
    float penetration;
    Vect normal;

};


void position_correction(Body& a, Body& b, Manifold& m){
    const float slop = 0.01; // usually 0.01 to 0.1
    const float percent = 0.2; // usually 20% to 80%
    Vect correction = (std::max(m.penetration-slop,0.0f) / (a.get_inv_mass() + b.get_inv_mass()) * percent) * m.normal;
    Vect a_pos = a.get_position();
    Vect a_correct = a.get_inv_mass()*correction;
    Vect new_a = a_pos-a_correct ;
    a.set_position(new_a);


    Vect b_pos = a.get_position();
    Vect b_correct = a.get_inv_mass()*correction;
    Vect new_b = b_correct + b_pos;
    b.set_position(new_b);
}


void set_new_speeds(Body& a, Body& b, Manifold& m ){

    Vect centreLine = b.get_position() - a.get_position();
    centreLine.normalize();
    m.penetration = get_depth(b.get_circle(), a.get_circle());
    m.normal = centreLine;
    std::cout<< "a pos " << a.get_position().get_x() << " " << a.get_position().get_y() << "\n";
    std::cout<< "b pos " << b.get_position().get_x() << " " << b.get_position().get_y() << "\n";
    std::cout << "prev  v_x a: " << a.get_velocity().get_x()<<"\n";
    std::cout << "prev  v_x b: " << b.get_velocity().get_x()<<"\n";
    std::cout << "prev  v_y a: " << a.get_velocity().get_y()<<"\n";
    std::cout << "prev  v_y b: " << b.get_velocity().get_y()<<"\n";

    //these are the initial velocity compnenets along the centreline of the 2 circles
    float initial_speed_a = dotProd(a.get_velocity(),centreLine);
    float initial_speed_b = dotProd(b.get_velocity(),centreLine);

    float e = std::min(a.get_e(), b.get_e());
    //along the collision normal, the problem becomes 1D
    float final_speed_b = (-e*(initial_speed_b - initial_speed_a) + a.get_mass()*initial_speed_a + b.get_mass()*initial_speed_b) * 1/(1 + b.get_mass());
    float final_speed_a = e*(initial_speed_b - initial_speed_a) + final_speed_b;

    //get full velocity by adding to the full velocity vector
    Vect a_along_n = scalar_mult(final_speed_a,centreLine);
    Vect final_velocity_a = a.get_velocity() +  a_along_n;
    Vect b_along_n = scalar_mult(final_speed_b,centreLine);
    Vect final_velocity_b = b.get_velocity() +  b_along_n;

   a.set_velocity(final_velocity_a.get_x(),final_velocity_a.get_y());
    b.set_velocity(final_velocity_b.get_x(),final_velocity_b.get_y());
    std::cout << "final  v_x a: " << a.get_velocity().get_x()<<"\n";
    std::cout << "final  v_x b: " << b.get_velocity().get_x()<<"\n";
    std::cout << "final  v_y a: " << a.get_velocity().get_y()<<"\n";
    std::cout << "final  v_y b: " << b.get_velocity().get_y()<<"\n";


}


