//
// Created by julia on 27/06/22.
//

#include "State.h"

void State::add_body(Body* body){
    for(Body* bod : bodies){
        if(bod == body){
            return;
        }
    }
    bodies.push_back(body);
}

void State::update_physics(double dt){

    for(Body* body : bodies){
        body->intersecting = false;
    }
    if(bodies.size() > 1) phase.generate_pairs();
    std::vector<Pair> pairs  = phase.get_pairs();
    std::vector<Manifold> ms;
    for(Pair pair : pairs){
        if(pair.a->get_shape().intersects(pair.b->get_shape()) || pair.b->get_shape().intersects(pair.a->get_shape())) {
            Body &aa = *pair.a;
            Body &bb = *pair.b;
            aa.intersecting = true;
            bb.intersecting = true;

            double e = std::min(aa.get_e(), bb.get_e());
            Vec collision_normal = bb.get_position() - aa.get_position();;
            collision_normal.normalize();
            Manifold m = Manifold{aa, bb, e, collision_normal};
            set_new_speeds(aa, bb, m,dt);
            //if(pair.a->mass != 0 || pair.b->mass != 0) position_correction(aa, bb, m);

            ms.push_back(m);
        }

    }for(Body* body : bodies){
        body->integrate(dt,w,h);
        body->normal = Vec{0,0};
        //body->intersecting = false;
        //   body->set_current(body->get_position());

        //   body->impulse = Vec{0,0};


    }
    for(Manifold m : ms){
        position_correction(m);
    }
    for(Body* body : bodies){
        //  body->integrate(dt,w,h);
        //  body->normal = Vec{0,0};
        body->set_current(body->get_position());

        body->impulse = Vec{0,0};


    }
    double width = w;
    auto end = std::remove_if(bodies.begin(),
                              bodies.end(),
                              [&width](Body* const &i) {
                                  return (i->get_position().get_y() < -500 || i->get_position().get_x() < -500 || i->get_position().get_x() > width + 500 ) ;    // remove odd numbers
                              });
    bodies.erase(end, bodies.end());



}

void State::set_render_pos(double a){

    for(Body* body : bodies){
        Vec render_pos{body->get_prev().get_x() * a + body->get_curr().get_x() * (1 - a), body->get_prev().get_y() * a + body->get_curr().get_y() * (1 - a)};
        body->set_render(render_pos);
        body->set_previous(body->get_curr());
    }
}


void State::reset(){
    bodies.erase(bodies.begin(), bodies.end());
    bodies.clear();
    AABB* ab = new AABB{Vec{w/8,h/4 - 100}, Vec{7*w/8,h/4}};
    Body* bod3 = new Body{*ab, 0.75, 0,Vec{0,0}};
    bodies.push_back(bod3);
}