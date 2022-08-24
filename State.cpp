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
        body->contacts.clear();


    }

    if(bodies.size() > 1) phase.generate_pairs();
    std::vector<Pair> pairs  = phase.get_pairs();

    for(Manifold* manifold : ms){
        manifold->stale = true;
    }

    for(Pair pair : pairs){
        if(pair.a->get_shape().intersects(pair.b->get_shape()) || pair.b->get_shape().intersects(pair.a->get_shape())) {
            Body &aa = *pair.a;
            Body &bb = *pair.b;
            aa.intersecting = true;
            bb.intersecting = true;


            auto* m = new Manifold{aa, bb };
            m->set_manifold();
            bool contains = false;
            for(Manifold* manifold : ms){
                if((pair.a == &manifold->a && pair.b == &manifold->b) || (pair.a == &manifold->b && pair.b == &manifold->a) ){
                //    m->set_manifold();
                    contains = true;
                    manifold->update(m);
                   // manifold->set_new_speeds(dt);
                    manifold->stale = false;
                    delete m;
                }
            }
            if(!contains){
              //  m->set_manifold();
                ms.push_back(m);
              //  m->set_new_speeds(dt);
                m->stale = false;
            }
          //  if(pair.a->mass != 0 || pair.b->mass != 0) position_correction( m);


        }



    }
    auto iter = std::remove_if(ms.begin(),
                              ms.end(),
                              [](Manifold* const &i) {
                                  return (i->stale) ;
                              });
    ms.erase(iter, ms.end());


    for(Body* body : bodies){
            Vec v{0, -dt * body->gravity };
            body->set_velocity(body->get_velocity() + v);
    }

    for(Manifold* m : ms){

       m->pre_step();
      // m->set_new_speeds(dt);
    }
    for(int i = 0; i< 10; i++){
        for(Manifold* mm : ms){
            mm->set_new_speeds(dt);
        }
    }

    for(Body* body : bodies){
       body->integrate(dt,w,h);
       // body->normal = Vec{0,0};
        //body->intersecting = false;
        //   body->set_current(body->get_position());

        //   body->impulse = Vec{0,0};


    }


    for(Manifold* m : ms){
        if(m->stale){

        }
        position_correction(*m);
    }
    for(Body* body : bodies){
        //  body->integrate(dt,w,h);
        //  body->normal = Vec{0,0};
        body->set_current(body->get_position());



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
    for(Manifold* m : ms){
        delete m;
    }
    ms.clear();
               OBB* obbe = new OBB{Vec{0,50},Vec{500,0}, Vec{w/2,h/4}};
    Body* oriented = new Body{*obbe,0,Vec{0,0}};
   OBB* obbe2 = new OBB{Vec{0,50},Vec{400,0}, Vec{w/2,h/2}};
   obbe2->rotate(10 * M_PI /180);
  Body* oriented2 = new Body{*obbe2,0,Vec{0,0}};
    bodies.push_back(oriented);
    bodies.push_back(oriented2);

}