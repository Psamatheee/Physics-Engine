//
// Created by julia on 19/08/22.
//

#ifndef ENGINE_MANIFOLD_H
#define ENGINE_MANIFOLD_H
#include "Collision.h"
struct Contact{
    Vec position;
    struct ID{
        bool operator==(ID other) const{
            return inc_edge1 == other.inc_edge1 && inc_edge2 == other.inc_edge2 && ref_edge1 == other.ref_edge1 && ref_edge2 == other.ref_edge2;
        }
        int inc_edge1 = 0;
        int inc_edge2 = 0;
        int ref_edge1 = 0;
        int ref_edge2 = 0;
    } id;

    double Pn = 0;
    double Pt = 0;
    double Pnb = 0;

};


struct Manifold{

    Manifold(Body& aa, Body& bb);

    void set_manifold();
    void set_new_speeds(double dt);
    void update(Manifold* m);
    void pre_step();
    void calculate_OBBvsCircle(Body& obb_body, Body& circle_body);

    Body& a;
    Body& b;
    double penetration;
    Vec normal;
    Contact contacts[2];
    int contact_num;
    bool stale = true;

};
void position_correction(Manifold &m);

#endif //ENGINE_MANIFOLD_H
