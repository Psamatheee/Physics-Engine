//
// Created by julia on 27/06/22.
//

#ifndef ENGINE_STATE_H
#define ENGINE_STATE_H


#include "DrawBody.h"


class BroadPhase {
public:
    explicit BroadPhase(std::vector<Body *> &bodies) : bodies(bodies) {
        pairs.clear();
    }

    void generate_pairs();
    std::vector<Pair> get_pairs() { return pairs; }

private:
    std::vector<Body *> &bodies;
    std::vector<Pair> pairs;
};

class State{
public:

    State(double ww, double hh) : w(ww), h(hh), phase(BroadPhase(bodies)){};

    std::vector<Body*> get_bodies() const {return bodies;}
    double get_h() const{return h;}

    void add_body(Body* body);
    void update_physics(double dt  );
    void set_render_pos(double a);
    void reset();

private:
    double w;
    double h;
    BroadPhase phase;
    std::vector<Body*> bodies;
    std::vector<Manifold*> ms;
};

struct DrawBodies{
    void update(State& state);
    std::vector<DrawBody> bodies;

};


#endif //ENGINE_STATE_H
