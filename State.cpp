//
// Created by julia on 27/06/22.
//

#include "State.h"

void State::add_body(Body *body) {
    for (Body *bod: bodies) {
        if (bod == body) {
            return;
        }
    }
    bodies.push_back(body);
}

void State::update_physics(double dt) {

    if (bodies.size() > 1) phase.generate_pairs();
    std::vector<Pair> pairs = phase.get_pairs();

    for (Manifold *manifold: ms) {
        manifold->stale = true;
    }

    for (Pair pair: pairs) {
        if (pair.a->get_shape().intersects(pair.b->get_shape()) ||
            pair.b->get_shape().intersects(pair.a->get_shape())) {
            Body &aa = *pair.a;
            Body &bb = *pair.b;

            auto *m = new Manifold{aa, bb};
            m->set_manifold();
            bool contains = false;
            for (Manifold *manifold: ms) {
                if ((pair.a == &manifold->a && pair.b == &manifold->b) ||
                    (pair.a == &manifold->b && pair.b == &manifold->a)) {
                    contains = true;
                    manifold->update(m);
                    manifold->stale = false;
                    delete m;
                }
            }
            if (!contains) {
                ms.push_back(m);
                m->stale = false;
            }

        }
    }

    auto iter = std::remove_if(ms.begin(),
                               ms.end(),
                               [](Manifold *const &i) {
                                   return (i->stale);
                               });
    ms.erase(iter, ms.end());

    for (Body *body: bodies) {
        Vec v{0, -dt * body->gravity};
        body->set_velocity(body->get_velocity() + v);
    }

    for (Manifold *m: ms) {
        m->pre_step();
    }

    for (int i = 0; i < 10; i++) {
        for (Manifold *mm: ms) {
            mm->set_new_speeds(dt);
        }
    }

    for (Body *body: bodies) {
        body->integrate(dt, w, h);
    }

    for (Manifold *m: ms) {
        position_correction(*m);
    }
    for (Body *body: bodies) {
        body->set_current(body->get_position());
    }

    double width = w;
    auto end = std::remove_if(bodies.begin(),
                              bodies.end(),
                              [&width](Body *const &i) {
                                  return (i->get_position().y < -500 || i->get_position().x < -500 ||
                                          i->get_position().x > width + 500);    // remove odd numbers
                              });
    bodies.erase(end, bodies.end());

}

void State::set_render_pos(double a) {

    for (Body *body: bodies) {
        Vec render_pos{body->get_prev().x * a + body->get_curr().x * (1 - a),
                       body->get_prev().y * a + body->get_curr().y * (1 - a)};
        body->set_render(render_pos);
        body->set_previous(body->get_curr());
    }
}

void State::reset() {
    bodies.erase(bodies.begin(), bodies.end());
    bodies.clear();

    for (Manifold *m: ms) {
        delete m;
    }

    ms.clear();
    OBB *obbe = new OBB{Vec{0, 50}, Vec{500, 0}, Vec{w / 2, h / 4}};
    Body *oriented = new Body{*obbe, 0, Vec{0, 0}};
    OBB *obbe2 = new OBB{Vec{0, 50}, Vec{400, 0}, Vec{w / 2, h / 2}};
    obbe2->rotate(10 * M_PI / 180);
    Body *oriented2 = new Body{*obbe2, 0, Vec{0, 0}};
    bodies.push_back(oriented);
    bodies.push_back(oriented2);

}