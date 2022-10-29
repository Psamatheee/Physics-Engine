//
// Created by julia on 27/06/22.
//

#ifndef ENGINE_STATE_H
#define ENGINE_STATE_H

#include "Body.h"
#include "Manifold.h"




class State{
public:


    State(double ww, double hh) : w(ww), h(hh){};

    std::vector<Body>& get_bodies()  {return bodies;}
    double get_h() const{return h;}

    void create_body(Type shape_type, Vec position);
    void add_body(Body& body);
    void update_physics(double dt  );
    void set_render_pos(double a);
    void reset();
    void generate_pairs();


private:
    double w;
    double h;
    std::vector<Body> bodies;
    std::vector<Manifold> ms;
    std::vector<Pair> pairs;
};




#endif //ENGINE_STATE_H
