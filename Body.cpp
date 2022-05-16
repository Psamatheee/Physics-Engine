//
// Created by julia on 05/05/22.
//

#include "Body.h"
//#include "Geometry.h"
#include <algorithm>
#include <iostream>
#include <utility>


void Body::integrate(double dt, double w, double h) {
    Vec dr{dt * velocity.get_x(), dt * velocity.get_y()};
    double new_x = get_position().get_x() + dr.get_x();
    double new_y = get_position().get_y() + dr.get_y();
    set_position(new_x , new_y);
    collides_wall(h,w);


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



bool Body::collides_wall(double h, double w){
    double pen;
    Vec normal{};
    Boundary bound = get_shape().collides_boundary(w,h);
    if(bound == Boundary::None) return false;
    switch (bound) {
        case Boundary::Top:
            set_velocity(get_velocity().get_x(), get_velocity().get_y() * -1);
            pen = get_position().get_y() + get_shape().get_radius() - h;
            normal.set_x(0);
            normal.set_y(1);
            break;
        case Boundary::Bottom:
            set_velocity(get_velocity().get_x(), get_velocity().get_y() * -1);
            pen = -get_position().get_y()+ get_shape().get_radius();
            normal.set_x(0);
            normal.set_y(-1);
            break;
        case Boundary::Right:
            set_velocity(get_velocity().get_x() * -1, get_velocity().get_y() );
            pen = get_position().get_x()+ get_shape().get_radius() - w;
            normal.set_x(1);
            normal.set_y(0);
            break;
        case Boundary::Left:
            set_velocity(get_velocity().get_x() * -1, get_velocity().get_y() );
            pen = -get_position().get_x()+ get_shape().get_radius();
            normal.set_x(-1);
            normal.set_y(0);
            break;

    }

        const double slop = 0.01; // usually 0.01 to 0.1
        const double percent = 0.2; // usually 20% to 80%

        Vec correction = (std::max(pen - slop, 0.0) / (get_inv_mass() ) * percent) * normal ;
        Vec a_pos = get_position();
        Vec a_correct = get_inv_mass() * correction;
        Vec new_a = a_pos - a_correct ;
        set_position(new_a);




    return true;
}

bool operator==(Body a, Body b) {
    bool vecs = (a.get_position().get_x() == b.get_position().get_x() && a.get_position().get_y() == b.get_position().get_y() && a.get_velocity().get_y() == b.get_velocity().get_y() &&a.get_velocity().get_x() == b.get_velocity().get_x()  );
    bool mass = (a.get_mass() == b.get_mass());
    return mass && vecs;
}

bool operator!=(Body a, Body b) {
    return !(a==b);
}

struct Manifold{
    Body& a;
    Body& b;
    double penetration;
    Vec normal;

};


void position_correction(Body& a, Body& b, Manifold& m){
    const double slop = 0.01; // usually 0.01 to 0.1
    const double percent = 0.2; // usually 20% to 80%
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


void set_new_speeds(Body& a, Body& b, Manifold& m ){
    Vec a_pos = a.get_position();
    Vec b_pos = b.get_position();
    Vec r = b_pos - a_pos;

    if(a.get_shape().get_type() == Type::Circle && b.get_shape().get_type() == Type::Circle) {
        Vec centreLine = b_pos - a_pos;

        centreLine.normalize();
        m.penetration = get_depth(b.get_shape(), a.get_shape());
        m.normal = centreLine;
        Vec relat_velocity = b.get_velocity() - a.get_velocity();
        if (dotProd(relat_velocity, centreLine) > 0) return;


        //these are the initial velocity compnenets along the centreline of the 2 circles
        double initial_speed_a = dotProd(a.get_velocity(), centreLine);
        double initial_speed_b = dotProd(b.get_velocity(), centreLine);

        double e = std::min(a.get_e(), b.get_e());
        double final_speed_b = a.get_mass() / (b.get_mass() + a.get_mass()) *
                               (initial_speed_a + b.get_mass() / a.get_mass() -
                                e * (initial_speed_b - initial_speed_a));
        double final_speed_a = e * (initial_speed_b - initial_speed_a) + final_speed_b;

        //get full velocity by adding to the full velocity vector
        Vec a_along_n = scalar_mult((-initial_speed_a + final_speed_a), centreLine);
        Vec final_velocity_a = a.get_velocity() + a_along_n;
        Vec b_along_n = scalar_mult(final_speed_b - initial_speed_b, centreLine);
        Vec final_velocity_b = b.get_velocity() + b_along_n;

        a.set_velocity(final_velocity_a.get_x(), final_velocity_a.get_y());
        b.set_velocity(final_velocity_b.get_x(), final_velocity_b.get_y());
    }

    if(a.get_shape().get_type() == Type::AABB && b.get_shape().get_type() == Type::AABB){
        Vec relat_velocity = b.get_velocity() - a.get_velocity();

        if (dotProd(relat_velocity, r) > 0) return;
       //already know it's intersecting
        double e = std::min(a.get_e(), b.get_e());
       Rectangle aa{a.get_shape().get_min(),a.get_shape().get_max()};
        Rectangle bb{b.get_shape().get_min(),b.get_shape().get_max()};
       if(r.get_x()<= 0){
           if(r.get_y() <= 0){
               //b intersects fully on left
               if(bb.min.get_y() > aa.min.get_y()){
                   m.normal = Vec{-1,0};
                   m.penetration = bb.max.get_x() - aa.min.get_x();
                   //b intersects fully on bottom
               }else if(bb.min.get_x() >= aa.max.get_x()){
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
               if(bb.min.get_x() >= aa.min.get_x()){
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
               if(bb.min.get_y() < aa.min.get_y() || x_pen<y_pen  ){
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

       if(m.normal.get_y() != 0){
         //  double final_b = 1/(a.get_mass() + b.get_mass()) * (a.get_mass()*a.get_velocity().get_y() + b.get_mass()*b.get_velocity().get_y()+a.get_mass()*e*(a.get_velocity().get_y() + b.get_velocity().get_y()));
         //  double final_a = final_b - e*(a.get_velocity().get_y() - b.get_velocity().get_y());
         //
           double final_speed_b = a.get_mass() / (b.get_mass() + a.get_mass()) *
                                  (a.get_velocity().get_y() + b.get_mass() / a.get_mass() -
                                   e * (b.get_velocity().get_y() - a.get_velocity().get_y()));
           double final_speed_a = e * (b.get_velocity().get_y() - a.get_velocity().get_y()) + final_speed_b;
           b.set_velocity(b.get_velocity().get_x(),final_speed_b);
            a.set_velocity(a.get_velocity().get_x(),final_speed_a);
            std::cout << "VELOCITY " << b.get_velocity().get_x() << " " << b.get_velocity().get_y() << "\n";
       }
        if(m.normal.get_x() != 0){
            double final_speed_b = a.get_mass() / (b.get_mass() + a.get_mass()) *
                                   (a.get_velocity().get_x() + b.get_mass() / a.get_mass() -
                                    e * (b.get_velocity().get_x() - a.get_velocity().get_x()));
            double final_speed_a = e * (b.get_velocity().get_x() - a.get_velocity().get_x()) + final_speed_b;
            b.set_velocity(final_speed_b,b.get_velocity().get_y());

            a.set_velocity(final_speed_a,a.get_velocity().get_y());
        }
    }




}

void calculate_manifold(Rectangle& a, Rectangle& b, Manifold& m){

}


class BroadPhase{
public:
   // explicit BroadPhase(std::vector<Body> bodies) : bodies(std::move(bodies)){}
    explicit BroadPhase(const std::vector<Body*>&  bodies) : bodies(bodies){
       pairs.clear();
   }
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
    const std::vector<Body*>& bodies;
    std::vector<Pair> pairs;
};


