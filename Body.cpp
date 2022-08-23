//
// Created by julia on 05/05/22.
//

#include "Body.h"
//#include "Geometry.h"
#include <algorithm>
#include <iostream>
#include <utility>


void Body::apply_impulse(Vec imp, Vec normal) {
    velocity  = velocity +  inv_mass * imp;
    double crosss = -cross( normal, imp );
    angular_vel += inv_inertia * crosss;
}

void Body::integrate(double dt, double w, double h) {
//    v += (1/m * F) * dt








//angle += angular_vel * dt;

   // set_velocity(ee);



    Vec dr{dt * velocity.get_x(), dt * velocity.get_y()};


    double new_x = get_position().get_x() + dr.get_x();
    double new_y = get_position().get_y() + dr.get_y();
    set_position(new_x , new_y);
    shape.rotate(angular_vel*dt);
    angle = shape.get_orient();


    double ek = 0.5 * mass * velocity.get_size()*velocity.get_size();
 //   Vec v{velocity.get_x(), velocity.get_y() - dt * gravity*1/2};
   // set_velocity(v);


    end_pos = get_position();





}



struct Pair{
    Body* a;
    Body* b;
};



bool operator==(Body a, Body b) {
    bool vecs = (a.get_position().get_x() == b.get_position().get_x() && a.get_position().get_y() == b.get_position().get_y() && a.get_velocity().get_y() == b.get_velocity().get_y() &&a.get_velocity().get_x() == b.get_velocity().get_x()  );
    bool mass = (a.get_mass() == b.get_mass());
    return mass && vecs;
}

bool operator!=(Body a, Body b) {
    return !(a==b);
}







// a is the rectangle




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
                    b.min = bodies[j]->get_shape().get_bounding_box().min  ;
                b.max = bodies[j]->get_shape().get_bounding_box().max  ;
                if(does_rect_intersect(a,b) && !(bodies[i]->sleep && bodies[j]->sleep) ){
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


