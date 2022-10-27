//
// Created by julia on 27/10/22.
//

#ifndef ENGINE_COLLISION_H
#define ENGINE_COLLISION_H
#include "Body.h"


class Collision {

};

double get_OBB_collision_normal(Body& a, Body& b, Edge& edge);
void set_incident_edge(Body& obb, Edge& edge, Vec& normal);
int clip(Vec normal, double dot_prod, Edge &edge, bool &clipped);
void calc_ref_inc_edges(Edge& ref, Edge& inc);
#endif //ENGINE_COLLISION_H
