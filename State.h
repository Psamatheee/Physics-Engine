//
// Created by julia on 27/06/22.
//

#ifndef ENGINE_STATE_H
#define ENGINE_STATE_H


#include "DrawBody.cpp"

class State{
public:

    State(double ww, double hh) : w(ww), h(hh), phase(BroadPhase(bodies)){};

    std::vector<Body*> get_bodies() const {return bodies;}
    double get_h() const{return h;}

    void add_body(Body* body);
    void update_physics(double dt, int& count);
    void set_render_pos(double a);
    void reset();

    int max_count = 100;

private:
    double w;
    double h;
    BroadPhase phase;
    std::vector<Body*> bodies;
    std::vector<Manifold*> ms;
};

struct DrawBodies{
    void update(State& state){
        std::vector<Body*> state_bodies = state.get_bodies();
        bodies.clear();
        for (Body* body : state_bodies){

            Body& bod = *body;
            DrawBody draw_body{bod, state.get_h(), sf::Color::White};
            bodies.push_back(draw_body);
        }
    }
    std::vector<DrawBody> bodies;

};


#endif //ENGINE_STATE_H
