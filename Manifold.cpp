//
// Created by julia on 19/08/22.
//

#include "Manifold.h"
#include "Geometry.h"
#include <algorithm>
#include <cfloat>


Manifold::Manifold(Body &aa, Body &bb) : a(aa), b(bb) {
    normal = Vec{};
    penetration = 0;
}

void Manifold::pre_step() {
    for (int i = 0; i < contact_num; i++) {
        Vec ra = contacts[i].position - a.get_position();
        Vec rb = contacts[i].position - b.get_position();
        Vec tangent = cross(normal, 1.0);
        tangent.normalize();
        Vec impulse = contacts[i].Pn * normal + contacts[i].Pt * tangent;
        a.apply_impulse(-1 * impulse, ra);
        b.apply_impulse(impulse, rb);
    }
}

void Manifold::update(Manifold *m) {
    Contact merged_contacts[2];
    for (int i = 0; i < m->contact_num; i++) {
        Contact newc = m->contacts[i];
        bool new_contact = true;
        for (int j = 0; j < contact_num; j++) {
            Contact oldc = contacts[j];
            if (newc.id == oldc.id && a.get_shape().get_type() != Type::Circle &&
                b.get_shape().get_type() != Type::Circle) {
                merged_contacts[i] = newc;
                merged_contacts[i].Pn = oldc.Pn;
                merged_contacts[i].Pt = oldc.Pt;
                merged_contacts[i].Pnb = oldc.Pnb;
                new_contact = false;
            }
        }
        if (new_contact) {
            merged_contacts[i] = newc;
        }

    }
    for (int i = 0; i < m->contact_num; i++) {
        contacts[i] = merged_contacts[i];
    }
    normal = m->normal;
    penetration = m->penetration;
    contact_num = m->contact_num;
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

void calculate_manifold_AABBvsCircle(AABB &a, Circle &b, Manifold &m) {
    Rectangle aa{a.get_min(), a.get_max()};
    Circle bb{b.get_radius(), b.get_position()};
    Vec closest = get_closest_point(aa, bb);
    double distance_sqr = pow(closest.x - bb.get_x(), 2) +
                          pow(closest.y - bb.get_y(), 2);
    double penetration = bb.get_radius() - sqrt(distance_sqr);
    m.penetration = penetration;

    Vec b_pos = b.get_position();
    Vec a_pos = closest;
    Vec centreLine = b_pos - a_pos;
    if (bb.get_centre().x < aa.max.x && bb.get_centre().x > aa.min.x &&
        bb.get_centre().y > aa.min.y && bb.get_centre().y < aa.max.y) {
        b.set_position(closest.x, closest.y);
        centreLine = Vec{centreLine.x * -1, centreLine.y * -1};
    }

    centreLine.normalize();
    m.normal = Vec{centreLine.x, centreLine.y};
    m.contacts[0] = Contact{(closest)};
}

void Manifold::calculate_OBBvsCircle(Body& obb_body, Body& circle_body) {
    double angle = -obb_body.get_shape().get_orient();
    Vec model_pos = circle_body.get_shape().get_position().rotate(angle);
    Circle circ{circle_body.get_shape().get_radius(), model_pos};
    Helper_Rect rect = obb_body.get_shape().get_points();
    AABB model_rect{rect.point3.rotate(angle), rect.point1.rotate(angle)};
    Manifold mm{a, b};
    calculate_manifold_AABBvsCircle(model_rect, circ, mm);
    Contact contact = mm.contacts[0];

    contact.position = contact.position.rotate(-angle);
    contacts[0] = contact;
    normal = mm.normal.rotate(-angle);
    contact_num = 1;
    penetration = mm.penetration;
}


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

double get_OBB_collision_normal(Body& bodyA, Body& bodyB, Edge& edge, int& edge_num){
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
        edge_num = 4;
    }
    if(bb < aa && bb < penetration){
        penetration = bb;
        edge[0] = a_rect[1];
        edge[1] = a_rect[2];
        edge_num = 2;
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
        edge_num = 1;
    }
    if(bb < aa && bb < penetration){
        penetration = bb;
        edge[0] = a_rect[2];
        edge[1] = a_rect[3];
        edge_num = 3;
    }

    return penetration;

}


void set_incident_edge(Body& obb, Edge& edge, Vec& normal, int& edge_num){
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
            edge_num = i+1;
        }
    }
}

void Manifold::set_manifold() {
    if ((a.get_shape().get_type() == Type::Circle && b.get_shape().get_type() == Type::Circle)) {
        normal = b.get_position() - a.get_position();
        normal.normalize();
        penetration = get_depth(a.get_shape(), b.get_shape());
        double alpha = std::abs(dotProd(b.get_position(), normal) - dotProd(a.get_position(), normal)) -
                       b.get_shape().get_radius();
        double x = a.get_shape().get_radius() - alpha;
        Contact contact;
        contact.position = a.get_position() + x / 2 * normal;
        contacts[0] = contact;
        contact_num = 1;
    }

    if (a.get_shape().get_type() == Type::OBB && b.get_shape().get_type() == Type::Circle) {
        calculate_OBBvsCircle(a,b);
        return;
    }
    if (b.get_shape().get_type() == Type::OBB && a.get_shape().get_type() == Type::Circle) {
        calculate_OBBvsCircle(b,a);
        normal = -1 * normal;
        return;
    }

    if (a.get_shape().get_type() == Type::AABB && b.get_shape().get_type() == Type::AABB) {
        Rectangle aa{a.get_shape().get_min(), a.get_shape().get_max()};
        Rectangle bb{b.get_shape().get_min(), b.get_shape().get_max()};
        Vec r = Vec{(bb.max.x + bb.min.x) / 2, (bb.max.y + bb.min.y) / 2} -
                Vec{(aa.max.x + aa.min.x) / 2, (aa.max.y + aa.min.y) / 2};
        double a_extent = (aa.max.x - aa.min.x) / 2;
        double b_extent = (bb.max.x - bb.min.x) / 2;
        double x_overlap = a_extent + b_extent - std::abs(r.x);

        // SAT test on x axis
        if (x_overlap > 0) {
            // Calculate half extents along x axis for each object
            a_extent = (aa.max.y - aa.min.y) / 2;
            b_extent = (bb.max.y - bb.min.y) / 2;

            // Calculate overlap on y axis
            double y_overlap = a_extent + b_extent - std::abs(r.y);

            // SAT test on y axis
            if (y_overlap > 0) {
                // Find out which axis is axis of least p
                if (x_overlap < y_overlap) {
                    // Point towards B knowing that n points from A to B
                    if (r.x < 0) normal = Vec{-1, 0};
                    else normal = Vec{1, 0};
                    penetration = x_overlap;

                } else {
                    // Point toward B knowing that n points from A to B
                    if (r.y < 0)
                        normal = Vec{0, -1};
                    else
                        normal = Vec{0, 1};
                    penetration = y_overlap;
                }
            }
        }
    }

    if (a.get_shape().get_type() == Type::OBB && b.get_shape().get_type() == Type::OBB) {
        contact_num = 0;
        double p;
        Vec n{};


        Edge edgeA;
        int edge_A_num = 0;
        Edge edgeB;
        Edge ref;

        int edge_B_num = 0;
        Edge inc;
        int ref_num, inc_num;

        double p_a = get_OBB_collision_normal(a, b, edgeA, edge_A_num);
        double p_b = get_OBB_collision_normal(b, a, edgeB, edge_B_num);

        bool flip;
        if (p_a <= p_b) {
            //a has the reference face
            p = p_a;
            ref = edgeA;
            inc = edgeB;
            ref_num = edge_A_num;
            flip = false;
        }
        if (p_b < p_a) {
            //b has the reference face
            ref = edgeB;
            ref_num = edge_B_num;
            inc = edgeA;
            p = p_b;
            flip = true;
        }

        Vec temp = ref[1] - ref[0];
        n.x = temp.y * -1;
        n.y = temp.x;
        n.normalize();
        normal = n;
        !flip ? set_incident_edge(b.get_shape(), edgeB, normal, inc_num) : set_incident_edge(a.get_shape(), edgeA, normal, inc_num);
        inc = !flip ? edgeB : edgeA;

        Contact c1, c2;
        c1.position = inc[0];
        c1.id.inc_edge2 = inc_num;
        c1.id.inc_edge1 = inc_num == 1 ? 4 : inc_num - 1;
        c2.position = inc[1];
        c2.id.inc_edge1 = inc_num;
        c2.id.inc_edge2 = inc_num == 4 ? 1 : inc_num + 1;


        penetration = p;
        a.edge = edgeA;
        b.edge = edgeB;
        //clip the sides of the reference face

        normal = n;
        penetration = p;

        //clip the sides of the reference face
        Vec side_normal = ref[1] - ref[0];
        int side_edge_num = ref_num == 4 ? 1 : ref_num + 1;

        side_normal.normalize();
        Vec l = ref[0];
        Vec r = ref[1];
        bool clipped = false;
        bool flip_contacts = c1.position.x < c2.position.x;
        if (ref[0].x > ref[1].x) {
            l = ref[1];
            r = ref[0];
            side_normal = -1 * side_normal;
            side_edge_num = (side_edge_num + 2) % 4;
            if (side_edge_num == 0) side_edge_num = 4;
        }
        double left = dotProd(-1 * side_normal, l);
        double right = dotProd(side_normal, r);

        if (clip(side_normal, right, inc, clipped) < 2) {
            penetration = 0;
            return;
        }
        if (clipped) {
            Contact &c_right = flip_contacts ? c2 : c1;
            if (c_right.id.inc_edge1 != inc_num) {
                c_right.id.inc_edge1 = inc_num;
                c_right.id.inc_edge2 = 0;
                c_right.id.ref_edge1 = 0;
                c_right.id.ref_edge2 = side_edge_num;


            } else {
                c_right.id.inc_edge1 = 0;
                c_right.id.inc_edge2 = inc_num;
                c_right.id.ref_edge1 = side_edge_num;
                c_right.id.ref_edge2 = 0;
            }
            clipped = false;
        }
        if (clip(-1 * side_normal, left, inc, clipped) < 2) {

            penetration = 0;
            return;
        }
        if (clipped) {
            Contact &c_left = flip_contacts ? c1 : c2;
            int side_num = (side_edge_num + 2) % 4;
            if (side_num == 0) side_num = 4;
            if (c_left.id.inc_edge1 != inc_num) {
                c_left.id.inc_edge1 = 0;
                c_left.id.inc_edge2 = inc_num;
                c_left.id.ref_edge1 = side_num;
                c_left.id.ref_edge2 = 0;


            } else {
                c_left.id.inc_edge1 = inc_num;
                c_left.id.inc_edge2 = 0;
                c_left.id.ref_edge1 = 0;
                c_left.id.ref_edge2 = side_num;
            }
            clipped = false;
        }

        normal = n;

        //pick only points behind reference edge;
        double distance = dotProd(normal, ref[0]);
        double point1_dist = dotProd(inc[0], normal);
        double point2_dist = dotProd(inc[1], normal);
        int count = 0;
        if (point1_dist < distance) {
            c1.position = inc[0];
            contacts[0] = c1;
            penetration = distance - point1_dist;
            count++;
        } else {
            penetration = 0;
        }
        if (point2_dist < distance) {
            c2.position = inc[1];
            contacts[count] = c2;
            penetration += distance - point2_dist;
            count++;
        }
        if (count) {

            penetration /= count;
        }

        if (flip) normal = -1 * normal;
        contact_num = count;
    }
}

void position_correction(Manifold &m) {

    const double slop = 0.1;
    const double percent = 0.4;


    Vec correction =
            (std::max(std::abs(m.penetration) - slop, 0.0) / (m.a.get_inv_mass() + m.b.get_inv_mass()) * percent) *
            m.normal;

    Vec a_pos = m.a.get_position();
    Vec a_correct = m.a.get_inv_mass() * correction;
    Vec new_a = a_pos - a_correct;
    m.a.set_position(new_a);

    Vec b_pos = m.b.get_position();
    Vec b_correct = m.b.get_inv_mass() * correction;
    Vec new_b = b_correct + b_pos;
    m.b.set_position(new_b);

}

void Manifold::set_new_speeds(double dt) {

    for (int i = 0; i < contact_num; i++) {
        Vec ra = contacts[i].position - a.get_position();
        Vec rb = contacts[i].position - b.get_position();
        Vec relative_vel = b.get_velocity() + cross(rb, b.angular_vel) - a.get_velocity() - cross(ra, a.angular_vel);
        double inverse_mass = a.inv_mass + b.inv_mass + pow(cross(ra, normal), 2) * a.inv_inertia +
                              pow(cross(rb, normal), 2) * b.inv_inertia;
        double j = -1 * (dotProd(relative_vel, normal));
        j = j / inverse_mass;

        double pn0 = contacts[i].Pn;
        contacts[i].Pn = std::max(pn0 + j, 0.0);
        j = contacts[i].Pn - pn0;

        Vec impulse = j * normal;
        a.apply_impulse(-1 * impulse, ra);
        b.apply_impulse(impulse, rb);

        relative_vel = b.get_velocity() + cross(rb, b.angular_vel) - a.get_velocity() - cross(ra, a.angular_vel);
        Vec tangent = cross(normal, 1.0);
        tangent.normalize();

        inverse_mass = a.inv_mass + b.inv_mass + pow(cross(ra, tangent), 2) * a.inv_inertia +
                       pow(cross(rb, tangent), 2) * b.inv_inertia;
        double jtt = -dotProd(relative_vel, tangent);
        double jt = jtt / inverse_mass;

        double dynamic_coefficient = std::sqrt(a.dynamic_coeff * b.dynamic_coeff);

        double maxPt = dynamic_coefficient * contacts[i].Pn;
        double oldtimp = contacts[i].Pt;
        contacts[i].Pt = std::max(-maxPt, std::min(oldtimp + jt, maxPt));
        jt = contacts[i].Pt - oldtimp;
        Vec friction_force = jt * tangent;

        b.apply_impulse(friction_force, rb);
        a.apply_impulse(-1 * friction_force, ra);

    }
}