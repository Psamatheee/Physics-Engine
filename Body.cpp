//
// Created by julia on 05/05/22.
//

#include "Body.h"
//#include "Geometry.h"
#include <algorithm>
#include <iostream>
#include <utility>


void Body::integrate(double dt, double w, double h) {
//    v += (1/m * F) * dt
double friction = 5;

    Vec v{velocity.get_x() ,velocity.get_y() - dt*gravity };


    set_velocity(v);



    Vec dr{dt * velocity.get_x(), dt * velocity.get_y()};

    double new_x = get_position().get_x() + dr.get_x();
    double new_y = get_position().get_y() + dr.get_y();
    set_position(new_x , new_y);
    double ek = 0.5 * mass * velocity.get_size()*velocity.get_size();







}

bool does_intersect(Body& a, Body& b){
    return a.shape.intersects(b.shape);
};

struct Pair{
    Body* a;
    Body* b;
};

bool compare(Pair& l, Pair& r ){
    return pow(l.a->get_position().get_x() - l.b->get_position().get_x(),2) + pow(l.a->get_position().get_y() - l.b->get_position().get_y(),2) < pow(r.a->get_position().get_x() - r.b->get_position().get_x(),2) + pow(r.a->get_position().get_y() - r.b->get_position().get_y(),2);
}



bool Body::collides_wall(double h, double w, double dt){
    double pen;
    double x_pen;
    double y_pen;
    Vec normal{};
    double friction = 0;
    Boundary bound = get_shape().collides_boundary(w,h);
    double slop = 0.01;
    switch (bound) {
        case Boundary::None:
            return false;
        case Boundary::Top:
            set_velocity(get_velocity().get_x(), get_velocity().get_y() * rest_const * -1);
            if(shape.get_type()==Type::Circle){
                set_position(get_position().get_x(), h-shape.get_radius() - slop);
            }
            if(shape.get_type()== Type::AABB){
                set_position(get_position().get_x(), h-slop);
            }
            normal.set_x(0);
            normal.set_y(1);
            break;
        case Boundary::Bottom:

            set_velocity(get_velocity().get_x(), get_velocity().get_y() * rest_const * -1);
        if(shape.get_type()==Type::Circle){
            set_position(get_position().get_x(),get_shape().get_radius() + 0.01);
            return true;
        }
        if(shape.get_type()== Type::AABB) {
            set_position(get_shape().get_max().get_x(),get_shape().get_max().get_y()-get_shape().get_min().get_y() + 0.01);
            return true;
        }
            normal.set_x(0);
            normal.set_y(1);

            break;
        case Boundary::Right:
            set_velocity(get_velocity().get_x() * -1 * rest_const, get_velocity().get_y() );
            if(shape.get_type()==Type::Circle){
                set_position(w-shape.get_radius()-slop,get_position().get_y());
            }
            if(shape.get_type()== Type::AABB){
                set_position(w - slop, get_position().get_y());
            }
            normal.set_x(1);
            normal.set_y(0);
            break;
        case Boundary::Left:
            set_velocity(get_velocity().get_x() * -1 * rest_const, get_velocity().get_y() );
            if(shape.get_type()==Type::Circle){
                set_position(shape.get_radius()+slop,get_position().get_y());
            }
            if(shape.get_type()== Type::AABB){
                set_position(shape.get_max().get_x()-shape.get_min().get_x() + slop, get_position().get_y());
            }
            normal.set_x(-1);
            normal.set_y(0);
            break;
        case Boundary::TL:
            set_velocity(get_velocity().get_x() * -1 * rest_const, get_velocity().get_y() * rest_const * -1 );
            if(shape.get_type()==Type::Circle){
                set_position(shape.get_radius()+slop, h-shape.get_radius() - slop);
            }
            if(shape.get_type()== Type::AABB){
                set_position(shape.get_max().get_x()-shape.get_min().get_x() + slop, h-slop);
            }
            normal.set_x(-1);
            normal.set_y(1);
            normal.normalize();
            break;
        case Boundary::BL:
            set_velocity(get_velocity().get_x() * -1 * rest_const, get_velocity().get_y()* rest_const * -1 );
            if(shape.get_type()==Type::Circle){
                set_position(shape.get_radius()+slop,get_shape().get_radius() + 0.01);
                return true;
            }
            if(shape.get_type()== Type::AABB) {
                set_position(shape.get_max().get_x()-shape.get_min().get_x() + slop,get_shape().get_max().get_y()-get_shape().get_min().get_y() + 0.01);
                return true;
            }
            normal.set_x(-1);
            normal.set_y(-1);
           normal.normalize();
            break;
        case Boundary::BR:
            set_velocity(get_velocity().get_x() * -1 * rest_const, get_velocity().get_y()* rest_const * -1 );
            if(shape.get_type()==Type::Circle){
                set_position(w-shape.get_radius()-slop,get_shape().get_radius() + 0.01);
                return true;
            }
            if(shape.get_type()== Type::AABB) {
                set_position(w - slop,get_shape().get_max().get_y()-get_shape().get_min().get_y() + 0.01);
                return true;
            }
            normal.set_x(1);
            normal.set_y(-1);
         normal.normalize();
            break;
        case Boundary::TR:
            set_velocity(get_velocity().get_x() * -1 * rest_const, get_velocity().get_y() * rest_const * -1 );
            if(shape.get_type()==Type::Circle){
                set_position(w-shape.get_radius()-slop,h-shape.get_radius() - slop);
                return true;
            }
            if(shape.get_type()== Type::AABB) {
                set_position(w - slop,h-slop);
                return true;
            }
            normal.set_x(1);
            normal.set_y(1);
           normal.normalize();
            break;


    }






    return false;
}

bool operator==(Body a, Body b) {
    bool vecs = (a.get_position().get_x() == b.get_position().get_x() && a.get_position().get_y() == b.get_position().get_y() && a.get_velocity().get_y() == b.get_velocity().get_y() &&a.get_velocity().get_x() == b.get_velocity().get_x()  );
    bool mass = (a.get_mass() == b.get_mass());
    return mass && vecs;
}

bool operator!=(Body a, Body b) {
    return !(a==b);
}

Body::~Body() {


}

struct Manifold{
    Body& a;
    Body& b;
    double penetration;
    Vec normal;

};


void position_correction(Body& a, Body& b, Manifold& m){

    const double slop = 0.01; // usually 0.01 to 0.1
    const double percent = 0.5; // usually 20% to 80%
    Vec correction = (std::max(m.penetration - slop, 0.0) / (a.get_inv_mass() + b.get_inv_mass()) * percent) * m.normal;
    Vec a_pos = a.get_position();
    Vec a_correct = a.get_inv_mass() * correction;
    Vec new_a = a_pos - a_correct ;
    a.set_position(new_a);


    Vec b_pos = b.get_position();
    Vec b_correct = b.get_inv_mass() * correction;
    Vec new_b = b_correct + b_pos;
    b.set_position(new_b);




}

// a is the rectangle
void calculate_manifold_AABBvsCircle(Body& a, Body& b, Manifold& m){
    Rectangle aa{a.get_shape().get_min(), a.get_shape().get_max()};
    Circle bb{b.get_shape().get_radius(), b.get_shape().get_position()};
    Vec closest = get_closest_point(aa,bb);
    double distance_sqr = pow(closest.get_x() - bb.get_x(), 2) +
                          pow(closest.get_y() - bb.get_y(), 2);
    double penetration = bb.get_radius() - sqrt(distance_sqr);
    m.penetration = penetration;

    Vec b_pos = b.get_position();
    Vec a_pos = closest;
    Vec centreLine = b_pos - a_pos;
    if(bb.get_centre().get_x() < aa.max.get_x() && bb.get_centre().get_x() >aa.min.get_x() && bb.get_centre().get_y() > aa.min.get_y() && bb.get_centre().get_y() < aa.max.get_y()){
       b.set_position(closest);
       centreLine = Vec{centreLine.get_x()*-1, centreLine.get_y()*-1};

    }

    centreLine.normalize();
    m.normal = Vec{centreLine.get_x(), centreLine.get_y()};
}

void set_manifold(Body& a, Body& b, Manifold& m){
    if ((a.get_shape().get_type() == Type::Circle && b.get_shape().get_type() == Type::Circle)) {
        Vec a_position = a.get_position();
        Vec b_position = b.get_position();
        Vec normal = b_position - a_position;
        normal.normalize();
        m.normal = normal;
        m.penetration = get_depth(a.get_shape(),b.get_shape());
    }
    if(a.get_shape().get_type() == Type::AABB && b.get_shape().get_type() == Type::Circle){
        calculate_manifold_AABBvsCircle(a,b,m);
    }
    if(b.get_shape().get_type() == Type::AABB && a.get_shape().get_type() == Type::Circle){
        calculate_manifold_AABBvsCircle(b,a,m);
        m.normal = -1.0 * m.normal ;
    }
    if(a.get_shape().get_type() == Type::AABB && b.get_shape().get_type() == Type::AABB){
        Rectangle aa{a.get_shape().get_min(),a.get_shape().get_max()};
        Rectangle bb{b.get_shape().get_min(),b.get_shape().get_max()};
        Vec r = bb.max - aa.max;
        if(r.get_x()<= 0){
            if(r.get_y() <= 0){
                //b intersects fully on left
                if(bb.min.get_y() > aa.min.get_y()){
                    m.normal = Vec{-1,0};
                    m.penetration = bb.max.get_x() - aa.min.get_x();
                    //b intersects fully on bottom
                }else if(bb.min.get_x() > aa.max.get_x()){
                    m.normal = Vec{0,-1};
                    m.penetration = bb.max.get_y() - aa.min.get_y();
                    //b intersects left and bottom
                }else{
                    double x_pen = bb.max.get_x() - aa.min.get_x();
                    double y_pen = bb.max.get_y() - aa.min.get_y();
                    if(x_pen < y_pen) {
                        m.penetration = x_pen;
                        m.normal = Vec{-1,0};
                    }else{
                        m.penetration = y_pen;
                        m.normal = Vec{0,-1};
                    }
                }
            }else{
                //b intersect fully top
                if(bb.min.get_x() > aa.min.get_x()){
                    m.penetration = aa.max.get_y() - bb.min.get_y();
                    m.normal = Vec{0,1};
                }else{
                    double x_pen = bb.max.get_x() - aa.min.get_x();
                    double y_pen = aa.max.get_y() - bb.min.get_y();
                    if(bb.min.get_y() < aa.min.get_y() || x_pen<y_pen){
                        m.penetration = x_pen;
                        m.normal = Vec{-1,0};
                    }else{
                        m.penetration = y_pen;
                        m.normal= Vec{0,1};
                    }
                }
            }
        } else{
            if(r.get_y() <= 0){
                double x_pen = aa.max.get_x()-bb.min.get_x();
                double y_pen = bb.max.get_y()-aa.min.get_y();
                if(bb.min.get_y() > aa.min.get_y() || x_pen<y_pen  ){
                    m.penetration = x_pen;
                    m.normal = Vec{1,0};
                }else{
                    m.penetration = y_pen;
                    m.normal = Vec{0,-1};
                }

            }else{
                double x_pen = aa.max.get_x() - bb.min.get_x();
                double y_pen = aa.max.get_y()-bb.min.get_y();
                if(bb.min.get_y() < aa.min.get_y() || x_pen < y_pen){
                    m.penetration = x_pen;
                    m.normal = Vec{1,0};
                }else{
                    m.penetration = y_pen;
                    m.normal = Vec{0,1};
                }
            }
        }
    }
}



void set_new_speeds(Body& a, Body& b, Manifold& m ){
    a.mass = a.back_up_mass;
    a.gravity = a.back_up_grav;
    b.mass = b.back_up_mass;
    b.gravity = b.back_up_grav;
    set_manifold(a, b, m);

    double Uab_normal = dotProd(b.get_velocity()-a.get_velocity(), m.normal); // initial relative velocity along the normal

    if(Uab_normal > 0) return; //moving away from each other
    double e = std::min(a.get_e(), b.get_e());

    //Have to do it this way so that will still work for 0 mass objects
    double j = (-1.0 * (1+e) * Uab_normal) / (a.get_inv_mass() + b.get_inv_mass());
    Vec impulse = j * m.normal;

    Vec a_change_impulse = a.get_inv_mass() * impulse;
    Vec b_change_impulse = b.get_inv_mass() * impulse;

    Vec a_velocity = a.get_velocity() - a_change_impulse;
    Vec b_velocity = b.get_velocity() + b_change_impulse;
 /*   if(a_change_impulse.get_y() < 21 &&  a_change_impulse.get_y() != 0){
        a_velocity.set_y(0);
        a.gravity = 0;
    }
    if(b_change_impulse.get_y() < 21 &&  b_change_impulse.get_y() != 0){
        b_velocity.set_y(0);
        b.gravity = 0;
    }*/
    a.set_velocity(a_velocity);
    b.set_velocity(b_velocity);

    //friction
    //Vec new_normal = b.se
    Vec tangent = Vec{m.normal.get_y()*-1, m.normal.get_x()};
    if(dotProd(b.get_velocity(),tangent) > 0) {
        tangent = -1*tangent;
    }
    tangent.normalize();
    Vec new_relative = b.get_velocity() - a.get_velocity();
    double jtt = dotProd(new_relative , tangent);
    double jt = jtt / (a.get_inv_mass() + b.get_inv_mass());

    double static_coefficient  = std::sqrt(a.static_coeff*a.static_coeff + b.static_coeff*b.static_coeff);
    double dynamic_coefficient  = std::sqrt(a.dynamic_coeff*a.dynamic_coeff + b.dynamic_coeff*b.dynamic_coeff);
    Vec friction_force;
    if(std::abs(jt) < impulse.get_size() *  static_coefficient){
        friction_force = std::abs(jt) * tangent;
    }else{
        friction_force = impulse.get_size() * dynamic_coefficient * tangent;
    }
    Vec a_change = a.get_inv_mass() * friction_force;
    Vec b_change = b.get_inv_mass() * friction_force;

    a_velocity = a.get_velocity() - a_change;
    b_velocity = b.get_velocity() + b_change;

    a.set_velocity(a_velocity);
    b.set_velocity(b_velocity);


    if(b.get_velocity().get_y()*b.get_velocity().get_y() < 100 && b.get_velocity().get_x() == 0){
        b.set_velocity(0,0);
        b.mass=0;
        b.gravity = 0;
    }
    if(a.get_velocity().get_y()*a.get_velocity().get_y() < 100 && a.get_velocity().get_x()){
        a.set_velocity(0,0);
        a.mass=0;
        a.gravity = 0;
    }

}





class BroadPhase{
public:
   // explicit BroadPhase(std::vector<Body> bodies) : bodies(std::move(bodies)){}
    explicit BroadPhase(std::vector<Body*>&  bodies) : bodies(bodies){
       pairs.clear();
   }
   void add_body(Body* bod){bodies.push_back(bod);}
    void generate_pairs(){
        pairs.clear();
        Rectangle a;
        Rectangle b;
        for(int i =0; i < bodies.size(); i++){
            int j = i + 1;
            a.min = bodies[i]->get_shape().get_bounding_box().min;
            a.max = bodies[i]->get_shape().get_bounding_box().max;
            while (j < bodies.size()){
                    b = bodies[j]->get_shape().get_bounding_box();
                    if(does_rect_intersect(a,b)){
                        Body* ap = bodies[i];
                        Body* bp = bodies[j];
                        pairs.push_back(Pair{ap,bp});
                    }
                    j++;
            }
        }


    }

    std::vector<Pair> get_pairs(){return pairs;}
private:
    std::vector<Body*>& bodies;
    std::vector<Pair> pairs;
};


