//
// Created by julia on 27/10/22.
//

#include <cfloat>
#include "Collision.h"
double get_min_point(Helper_Rect& rect, Vec& axis, Vec& point){
    double pen = FLT_MAX;
    point = Vec{};
    for(int i =0; i < 4; i++){
        if(dotProd(axis,rect[i]) < pen) {
            pen =  dotProd(axis,rect[i]);
            point = rect[i];
        }
    }
    return pen;
}
double get_max_point(Helper_Rect& rect, Vec& axis, Vec& point){
    double pen = 0;
    point = Vec{};
    for(int i =0; i < 4; i++){
        if(dotProd(axis,rect[i]) > pen) {
            pen =  dotProd(axis,rect[i]);
            point = rect[i];
        }
    }
    return pen;
}

double get_OBB_collision_normal(Body& bodyA, Body& bodyB, Edge& edge){
    Shape& a = bodyA.get_shape();
    Shape& b = bodyB.get_shape();
    Helper_Rect b_rect = b.get_points();
    Helper_Rect a_rect = a.get_points();

    double penetration = FLT_MAX;

    Vec n1 = a.get_max();
    Vec n2 = a.get_min();
    n1.normalize();
    n2.normalize();

    Vec max_a = a.get_position() + a.get_max() + a.get_min();
    Vec min_a = a.get_position() - a.get_min() - a.get_min();

    //n1
    Vec max_b{};
    Vec min_b{};
    get_max_point(b_rect, n1, max_b);
    get_min_point(b_rect, n1, min_b);
    double aa = dotProd(max_a, n1) - dotProd(min_b, n1);
    double bb = dotProd(max_b, n1) - dotProd(min_a, n1);

    if(aa < bb && aa < penetration){
        penetration = aa;
        edge[0] = a_rect[3];
        edge[1] = a_rect[0];
        edge.edge_num = 4;
    }
    if(bb < aa && bb < penetration){
        penetration = bb;
        edge[0] = a_rect[1];
        edge[1] = a_rect[2];
        edge.edge_num = 2;
    }

    //n2
    get_max_point(b_rect, n2, max_b);
    get_min_point(b_rect, n2, min_b);
    aa = dotProd(max_a, n2) - dotProd(min_b, n2);
    bb = dotProd(max_b, n2) - dotProd(min_a, n2);

    if(aa < bb && aa < penetration){
        penetration = aa;
        edge[0] = a_rect[0];
        edge[1] = a_rect[1];
        edge.edge_num = 1;
    }
    if(bb < aa && bb < penetration){
        penetration = bb;
        edge[0] = a_rect[2];
        edge[1] = a_rect[3];
        edge.edge_num = 3;
    }

    return penetration;

}


void set_incident_edge(Body& obb, Edge& edge, Vec& normal){
    Shape& box = obb.get_shape();
    Helper_Rect rect = box.get_points();
    double d = 1;
    for(int i = 0; i < 4; i++){
        Vec edge_normal = i != 3? rect[i] - rect[i+1] : rect[i] - rect[0];
        edge_normal = edge_normal.orthogonalize();
        double dot = dotProd(edge_normal, normal);
        if( dot <= d) {
            d = dot;
            edge.point1 = rect[i];
            edge.point2 = i != 3? rect[i+1] : rect[0];
            edge.edge_num = i+1;
        }
    }
}

int clip(Vec normal, double dot_prod, Edge &edge, bool &clipped) {

    double d1 = dotProd(normal, edge[0]) - dot_prod;
    double d2 = dotProd(normal, edge[1]) - dot_prod;
    int count = 0;
    if (d1 < 0) count++;
    if (d2 < 0) count++;

    //clip 1st point
    if (d1 * d2 < 0) {
        clipped = true;
        if (d1 >= 0) {
            edge[0] = edge[0] + d1 / (d1 - d2) * (edge[1] - edge[0]);
            count++;
        } else if (d2 >= 0) {
            edge[1] = edge[0] + d1 / (d1 - d2) * (edge[1] - edge[0]);
            count++;

        }
    }
    if (count > 2) {
        int k = 0;
    }
    return count;
}

void calc_ref_inc_edges(Body& a, Body& b, Edge& ref, Edge& inc){

}
