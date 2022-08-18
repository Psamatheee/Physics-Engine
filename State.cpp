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

void State::update_physics(double dt, int& count){

    for(Body* body : bodies){
        body->intersecting = false;
        body->contacts.clear();
        if(count == 1) body->start_pos = body->get_position();
        if(count == max_count){
            Vec displacement = body->end_pos - body->start_pos;
           if(std::abs(displacement.get_size()) < 2 ) {body->sleep = true;
           body->set_velocity(0,0);
           body->angular_vel = 0;
           }
        }

    }
    for(Body* body : bodies){
        if(!body->sleep) {
            Vec v{0, -dt * body->gravity * 1 / 2};
            body->set_velocity(body->get_velocity() + v);
        }
    }
    if(bodies.size() > 1) phase.generate_pairs();
    std::vector<Pair> pairs  = phase.get_pairs();
    std::vector<Manifold*> ms;

    for(Pair pair : pairs){
        if(pair.a->get_shape().intersects(pair.b->get_shape()) || pair.b->get_shape().intersects(pair.a->get_shape())) {
            Body &aa = *pair.a;
            Body &bb = *pair.b;
            aa.intersecting = true;
            bb.intersecting = true;

            double e = std::min(aa.get_e(), bb.get_e());
            Vec collision_normal = bb.get_position() - aa.get_position();;
            collision_normal.normalize();
            auto* m = new Manifold{aa, bb, 0, collision_normal};
          //  if(pair.a->mass != 0 || pair.b->mass != 0) position_correction( m);

            ms.push_back(m);
            if(!(aa.sleep && bb.sleep))set_new_speeds(*m, dt);
        }



    }


    for(Manifold* m : ms){

       // set_new_speeds(m,dt);
    }

    for(Body* body : bodies){
       if(!body->sleep) body->integrate(dt,w,h);
       // body->normal = Vec{0,0};
        //body->intersecting = false;
        //   body->set_current(body->get_position());

        //   body->impulse = Vec{0,0};


    }
    for(Manifold* m : ms){
        position_correction(*m);
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

    for(Manifold* m : ms){
        delete m;
    }



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
    Body* bod3 = new Body{*ab, 0,Vec{0,0}};
    bodies.push_back(bod3);
}