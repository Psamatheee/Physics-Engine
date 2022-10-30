//
// Created by julia on 27/06/22.
//

#include <random>
#include <vector>
#include <algorithm>
#include "State.h"
#include "Body.h"

std::random_device rd;
std::mt19937 mt(rd());
std::uniform_int_distribution<int> gen(10,80);

/* Broad phase of the physics update step.
   Collects pairs of bodies that are potentially intersecting by testing if their bounding
   boxes are intersecting
*/
void State::generate_pairs() {
    pairs.clear();
    Rectangle a;
    Rectangle b;
    for (int i = 0; i < bodies.size(); i++) {
        int j = i + 1;
        a.min = bodies[i].shape->get_bounding_box().min;
        a.max = bodies[i].shape->get_bounding_box().max;
        while (j < bodies.size()) { //makes sure not to include duplicate pairs
            b.min = bodies[j].shape->get_bounding_box().min;
            b.max = bodies[j].shape->get_bounding_box().max;
            if ( does_rect_intersect(a, b) ) {
                Body& ap = bodies[i];
                Body& bp = bodies[j];
                pairs.push_back(Pair{ap, bp});
            }
            j++;
        }
    }
}

void State::create_body(Type shape_type, Vec position) {
    if(shape_type == Type::Circle){
        double i = gen(mt);
        std::unique_ptr<Circle> circ_shape = std::make_unique<Circle>(Circle{ i, position});
        bodies.emplace_back(Body{std::move(circ_shape)});
    }
    if(shape_type == Type::OBB){
        double half_height = gen(mt);
        double half_width = gen(mt);

        std::unique_ptr<OBB> obb = std::make_unique<OBB>(OBB{Vec{0, half_height}, Vec{half_width, 0}, position});
        obb->rotate(gen(mt));
        bodies.emplace_back(Body{std::move(obb)});
    }
}

void State::update_physics(double dt) {

    for (Manifold& manifold: ms) {
        manifold.stale = true;
    }

    if (bodies.size() > 1) {
        generate_pairs(); // possibly intersecting bodies
        for (Pair& pair: pairs) {

            if (is_intersecting(pair.a,pair.b)) {
                Body& aa = pair.a;
                Body& bb = pair.b;
                Manifold m{aa, bb};
                m.set_manifold();
                bool contains = false;
                for (Manifold& manifold: ms) {
                    //check if already have manifold for this pair of bodies, and if so update manifold
                    if ((&aa == &manifold.a && &pair.b == &manifold.b) ||
                        (&pair.a == &manifold.b && &pair.b == &manifold.a)) {
                        contains = true;
                        manifold.update(m);
                        manifold.stale = false;
                    }
                }
                if (!contains) {
                    m.stale = false;
                    ms.push_back(m);
                }

            }
        }
    }

    //only include manifolds for currently intersecting objects
    std::vector<Manifold> manifolds;
    for(Manifold& m : ms){
            if(!m.stale) manifolds.push_back(m);
    }
    ms.clear();
    ms = std::move(manifolds);

    for (Body& body: bodies) {
        Vec v{0, -dt * body.gravity};
        body.velocity = body.velocity + v;
    }

    for (Manifold& m: ms) {
        m.pre_step();
    }

    for (int i = 0; i < 10; i++) {
        for (Manifold& mm: ms) {
            mm.set_new_speeds(dt);
        }
    }

    for (Body& body: bodies) {
        body.integrate(dt);
    }

    for (Manifold& m: ms) {
        position_correction(m);
    }

    //delete all the bodies that have gone far enough off the screen
    double width = w;
    double screen_limit = 500;
    auto end = std::remove_if(bodies.begin(),
                              bodies.end(),
                              [&width, &screen_limit](Body const &i) {
                                  return (i.get_position().y < -screen_limit || i.get_position().x < -screen_limit ||
                                          i.get_position().x > width + screen_limit);
                              });
    bodies.erase(end, bodies.end());
}

void State::set_render_pos(double a) {

    for (Body& body: bodies) {
        Vec render_pos{body.previous.x * a + body.current.x * (1 - a),
                       body.previous.y * a + body.current.y * (1 - a)};
        body.render_position = render_pos;
        body.previous = body.current;
    }
}

void State::reset() {
    bodies.clear();
    ms.clear();
    std::unique_ptr<OBB> obb( new OBB{Vec{0, 50}, Vec{700, 0}, Vec{w / 2, h / 4}});
    Body body{std::move(obb)};
    body.set_static();
    bodies.push_back(std::move(body));

   std::unique_ptr<OBB> obb2( new OBB{Vec{0, 10}, Vec{200, 0}, Vec{w / 4, h / 2}});
   obb2->rotate(10 * M_PI / 180);
   Body body2{std::move(obb2)};
   body2.set_static();
    bodies.push_back(std::move(body2));

}


