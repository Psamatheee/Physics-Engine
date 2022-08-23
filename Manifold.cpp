//
// Created by julia on 19/08/22.
//

#include "Manifold.h"
#include <algorithm>

Manifold::Manifold(Body &aa, Body &bb) : a(aa), b(bb){
    normal = Vec{};
    penetration = 0;

   // set_manifold();
}

void Manifold::pre_step() {
    for(int i = 0; i < contact_num; i++){

        Vec ra = contacts[i].position - a.get_position();
        Vec rb = contacts[i].position - b.get_position();
        Vec tangent =   cross(normal, 1.0);
        tangent.normalize();
        Vec impulse = contacts[i].Pn * normal + contacts[i].Pt * tangent;
        a.apply_impulse(-1 * impulse, ra);
        b.apply_impulse(impulse, rb);
    }
}

void Manifold::update(Manifold *m) {
    Contact merged_contacts[2];
    int new_count = 0;
    for(int i = 0; i < m->contact_num; i++){
        Contact newc = m->contacts[i];
        bool new_contact = true;
        for(int j =0; j<contact_num; j++){
            Contact oldc = contacts[j];
            if(newc.id == oldc.id && a.get_shape().get_type() != Type::Circle && b.get_shape().get_type() != Type::Circle ){
                merged_contacts[i] = newc;
                merged_contacts[i].Pn = oldc.Pn;
                merged_contacts[i].Pt = oldc.Pt;
                merged_contacts[i].Pnb = oldc.Pnb;
                new_contact = false;
            }
        }
        if(new_contact){
            merged_contacts[i] = newc;
        }

    }
    for(int i =0 ; i < m->contact_num; i++){
        contacts[i] = merged_contacts[i];
    }
    normal = m->normal;
    penetration = m->penetration;
    contact_num = m->contact_num;
}

int clip(Vec normal, double dot_prod, Edge& edge, bool& clipped){

    double d1 = dotProd(normal, edge[0]) - dot_prod;
    double d2 = dotProd(normal, edge[1]) - dot_prod;
    int count = 0;
    if(d1 < 0) count++;
    if(d2 < 0) count++;

    //clip 1st point
    if(d1 * d2 < 0){
        clipped = true;
        if(d1 >= 0){
            edge[0] = edge[0] + d1/(d1-d2) * (edge[1] - edge[0]);
            count++;
        }
        else if( d2 >= 0){
            edge[1] = edge[0] + d1/(d1-d2) * (edge[1] - edge[0]);
            count++;

        }
    }
    if(count >2){
        int k = 0;
    }
    return count;
}
void calculate_manifold_AABBvsCircle(AABB& a, Circle& b, Manifold& m){
    Rectangle aa{a.get_min(), a.get_max()};
    Circle bb{b.get_radius(), b.get_position()};
    Vec closest = get_closest_point(aa,bb);
    double distance_sqr = pow(closest.get_x() - bb.get_x(), 2) +
                          pow(closest.get_y() - bb.get_y(), 2);
    double penetration = bb.get_radius() - sqrt(distance_sqr);
    m.penetration = penetration;

    Vec b_pos = b.get_position();
    Vec a_pos = closest;
    Vec centreLine = b_pos - a_pos;
    if(bb.get_centre().get_x() < aa.max.get_x() && bb.get_centre().get_x() >aa.min.get_x() && bb.get_centre().get_y() > aa.min.get_y() && bb.get_centre().get_y() < aa.max.get_y()){
        b.set_position(closest.get_x(), closest.get_y());
        centreLine = Vec{centreLine.get_x()*-1, centreLine.get_y()*-1};
//
    }

    centreLine.normalize();
    m.normal = Vec{centreLine.get_x(), centreLine.get_y()};
    m.contacts[0] = Contact{(closest)};
}


void Manifold::set_manifold() {
    if ((a.get_shape().get_type() == Type::Circle && b.get_shape().get_type() == Type::Circle)) {
        Vec a_position = a.get_position();
        Vec b_position = b.get_position();
        normal = b_position - a_position;
        normal.normalize();
        penetration = get_depth(a.get_shape(),b.get_shape());
        double alpha = std::abs(dotProd(b.get_position(), normal) - dotProd(a.get_position(), normal)) - b.get_shape().get_radius();
        double x = a.get_shape().get_radius() - alpha;
        Contact contact;
        contact.position = a.get_position()  + x/2  * normal;
        contacts[0] = contact;
        contact_num = 1;
    }
    if(a.get_shape().get_type() == Type::AABB && b.get_shape().get_type() == Type::Circle){
        //  calculate_manifold_AABBvsCircle(a,b,m);
    }
    if(b.get_shape().get_type() == Type::AABB && a.get_shape().get_type() == Type::Circle){
        //   calculate_manifold_AABBvsCircle(b,a,m);
        normal = -1.0 * normal ;
    }

    if(a.get_shape().get_type() == Type::OBB && b.get_shape().get_type() == Type::Circle){
        double angle = -a.get_shape().get_orient();
        Vec og_pos = b.get_shape().get_position();
        double conv = M_PI / 180;
        double cos = std::cos((angle ));
        double sin = std::sin(angle );
        Matrix matrix{cos, sin, -sin, cos};
        Vec model_pos = matrix * og_pos;
        Circle circ{b.get_shape().get_radius(), model_pos};
        Helper_Rect rect = a.get_shape().get_points();
        AABB model_rect{matrix * rect.point3, matrix * rect.point1};
        Manifold mm{a,b};
        calculate_manifold_AABBvsCircle(model_rect, circ, mm);
        Contact contact = mm.contacts[0];
        cos = std::cos((-angle ));
        sin = std::sin(-angle );
        Matrix matrix2{cos, sin, -sin, cos};
        contact.position = matrix2 * contact.position;
        contacts[0] = contact;
        normal = matrix2 * mm.normal;
        contact_num = 1;
        penetration = mm.penetration;
        return;
    }

    if(b.get_shape().get_type() == Type::OBB && a.get_shape().get_type() == Type::Circle ){
        double angle = -b.get_shape().get_orient();
        Vec og_pos = a.get_shape().get_position();
        double conv = M_PI / 180;
        double cos = std::cos((angle ));
        double sin = std::sin(angle );
        Matrix matrix{cos, sin, -sin, cos};
        Vec model_pos = matrix * og_pos;
        Circle circ{a.get_shape().get_radius(), model_pos};
        Helper_Rect rect = b.get_shape().get_points();
        AABB model_rect{matrix * rect.point3, matrix * rect.point1};
        Manifold m{b,a};
        calculate_manifold_AABBvsCircle(model_rect, circ, m);
        Contact contact = m.contacts[0];
        cos = std::cos((-angle ));
        sin = std::sin(-angle );
        Matrix matrix2{cos, sin, -sin, cos};
        contact.position = matrix2 * contact.position;
        contacts[0] = contact;
        contact_num = 1;
        normal = matrix2 * m.normal;
        normal = -1 * normal;
        penetration = m.penetration;
        return;
    }

    if(a.get_shape().get_type() == Type::AABB && b.get_shape().get_type() == Type::AABB){
        Rectangle aa{a.get_shape().get_min(),a.get_shape().get_max()};
        Rectangle bb{b.get_shape().get_min(),b.get_shape().get_max()};
        Vec r = Vec{(bb.max.get_x() + bb.min.get_x())/2, (bb.max.get_y()+bb.min.get_y())/2} -Vec{(aa.max.get_x() + aa.min.get_x())/2, (aa.max.get_y()+aa.min.get_y())/2} ;
        //  Vec r = b.get_position() - a.get_position();
        double a_extent = (aa.max.get_x() - aa.min.get_x())/2;
        double b_extent = (bb.max.get_x() - bb.min.get_x())/2;
        double x_overlap = a_extent + b_extent - std::abs( r.get_x() );

        // SAT test on x axis
        if(x_overlap > 0)
        {
            // Calculate half extents along x axis for each object
            a_extent = (aa.max.get_y() - aa.min.get_y()) / 2;
            b_extent = (bb.max.get_y() - bb.min.get_y()) / 2;

            // Calculate overlap on y axis
            double y_overlap = a_extent + b_extent - std::abs( r.get_y() );

            // SAT test on y axis
            if(y_overlap > 0)
            {
                // Find out which axis is axis of least p
                if(x_overlap < y_overlap)
                {
                    // Point towards B knowing that n points from A to B
                    if(r.get_x() < 0) normal = Vec{-1, 0 };
                    else normal = Vec{ 1, 0 };
                    penetration = x_overlap;

                }
                else
                {
                    // Point toward B knowing that n points from A to B
                    if(r.get_y() < 0)
                        normal = Vec{ 0, -1 };
                    else
                        normal = Vec{ 0, 1 };
                    penetration = y_overlap;
                }
            }
        }
    }

    if(a.get_shape().get_type() == Type::OBB && b.get_shape().get_type() == Type::OBB ){
        contact_num = 0;
        double p;
        Vec n{};
        OBB aa{a.get_shape().get_max(), a.get_shape().get_min(), a.get_position()};
        aa.orient = a.get_shape().get_orient();
        OBB bb{b.get_shape().get_max(), b.get_shape().get_min(), b.get_position()};
        bb.orient = b.get_shape().get_orient();

        Edge edgeA;
        int edge_A_num = 0;
        Edge edgeB;
        Edge ref;

        int edge_B_num = 0;
        Edge inc;
        int ref_num,inc_num;

        double p_a = get_collision_normal(aa,bb,edgeA,edge_A_num);
        double p_b = get_collision_normal(bb,aa,edgeB, edge_B_num);

        bool flip;
        if(p_a <= p_b){
            //a has the reference face
            p = p_a;
            ref = edgeA;
            inc = edgeB;
            ref_num = edge_A_num;
            flip = false;
        }
        if(p_b < p_a){
            //b has the reference face
            ref = edgeB;
            ref_num = edge_B_num;
            inc = edgeA;
            p = p_b;
            flip = true;
        }

        Vec temp = ref[1] - ref[0];
        n.set_x(temp.get_y() * -1);
        n.set_y(temp.get_x());
        n.normalize();
        normal = n;
        !flip? set_incident_edge(bb,edgeB,normal,inc_num) : set_incident_edge(aa,edgeA, normal, inc_num);
        inc = !flip? edgeB : edgeA;

        Contact c1,c2;
        c1.position = inc[0];
        c1.id.inc_edge2 = inc_num;
        c1.id.inc_edge1 = inc_num == 1? 4: inc_num-1;
        c2.position = inc[1];
        c2.id.inc_edge1 = inc_num;
        c2.id.inc_edge2 = inc_num == 4? 1: inc_num+1;



        penetration = p;
        a.edge = edgeA;
        b.edge = edgeB;
        //clip the sides of the reference face

        normal = n;
        penetration = p;

        //clip the sides of the reference face
        Vec side_normal = ref[1] - ref[0];
        int side_edge_num = ref_num==4? 1: ref_num+1;

        side_normal.normalize();
        Vec l = ref[0];
        Vec r = ref[1];
        bool clipped = false;
        bool flip_contacts = c1.position.get_x() < c2.position.get_x();
        if(ref[0].get_x() > ref[1].get_x()){
            l = ref[1];
            r = ref[0];
            side_normal = -1*side_normal;
            side_edge_num = (side_edge_num + 2) % 4;
            if (side_edge_num == 0) side_edge_num = 4;
        }
        double left = dotProd(-1 * side_normal, l);
        double right = dotProd( side_normal, r);

        if( clip(side_normal, right, inc, clipped) < 2) {
            penetration = 0;
            return;
        }
        if(clipped){
            Contact& c_right = flip_contacts? c2 : c1;
            if(c_right.id.inc_edge1 != inc_num) {
                c_right.id.inc_edge1 = inc_num;
                c_right.id.inc_edge2 = 0;
                c_right.id.ref_edge1 = 0;
                c_right.id.ref_edge2 = side_edge_num;


            }else{
                c_right.id.inc_edge1 = 0;
                c_right.id.inc_edge2 = inc_num;
                c_right.id.ref_edge1 = side_edge_num;
                c_right.id.ref_edge2 = 0;
            }
            clipped = false;
        }
        if( clip(-1 * side_normal, left, inc, clipped) < 2) {

            penetration = 0;
            return;
        }
        if(clipped){
            Contact& c_left = flip_contacts? c1 : c2;
            int side_num = (side_edge_num + 2) % 4;
            if (side_num == 0) side_num = 4;
            if(c_left.id.inc_edge1 != inc_num) {
                c_left.id.inc_edge1 = 0;
                c_left.id.inc_edge2 = inc_num;
                c_left.id.ref_edge1 = side_num;
                c_left.id.ref_edge2 = 0;


            }else{
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
        double point1_dist = dotProd(inc[0],normal);
        double point2_dist = dotProd(inc[1],normal);
        int count = 0;
        if(point1_dist < distance){
            c1.position = inc[0];
            contacts[0] = c1;
            penetration = distance - point1_dist;
            count++;
        }else{
            penetration = 0;
        }
        if(point2_dist < distance){
            c2.position = inc[1];
            contacts[count] = c2;
            penetration += distance - point2_dist;
            count++;
        }
        if(count) {

            penetration /= count;
        }

        if(flip ) normal = -1 * normal;
        contact_num = count;
    }
}

void position_correction(Manifold& m){

    const double slop = 0.1; // usually 0.01 to 0.1
    const double percent = 0.4; // usually 20% to 80%
    //  if(m.a.get_shape().get_type() == Type::OBB || m.b.get_shape().get_type() == Type::OBB) return;
    if(m.penetration != m.penetration){
        int i =0 ;
    }
    Vec correction = (std::max(std::abs(m.penetration) - slop, 0.0) / (m.a.get_inv_mass() + m.b.get_inv_mass()) * percent) * m.normal;
    Vec a_pos = m.a.get_position();
    Vec a_correct = m.a.get_inv_mass() * correction;
    Vec new_a = a_pos - a_correct ;
    m.a.set_position(new_a);


    Vec b_pos = m.b.get_position();
    Vec b_correct = m.b.get_inv_mass() * correction;
    Vec new_b = b_correct + b_pos;
    m.b.set_position(new_b);




}




void set_manifold(Body& a, Body& b, Manifold& m){

}






void Manifold::set_new_speeds(double dt ){

    // m.penetration = 0;
   // Body& a  = m.a;
  //  Body& b  = m.b;
   // set_manifold(a, b, m);


    double e = std::min(a.get_e(), b.get_e());


    Vec og_v_b = b.get_velocity();
    Vec og_v_a = a.get_velocity();
    double og_w_b = b.angular_vel;
    double og_w_a = a.angular_vel;
    if(a.sleep) a.sleep = false;
    if(b.sleep) b.sleep = false;
    for(int i = 0; i < contact_num; i++){
        Vec ra = contacts[i].position - a.get_position();
        Vec rb = contacts[i].position - b.get_position();
        Vec relative_vel = b.get_velocity() + cross(rb,b.angular_vel) - a.get_velocity() -  cross(ra, a.angular_vel);
      //  if(dotProd(relative_vel,normal) > 0) return; //moving away from each other
        double inverse_mass = a.inv_mass + b.inv_mass + pow(cross(ra, normal) , 2)*a.inv_inertia + pow(cross(rb, normal) , 2)*b.inv_inertia;
        double bias = 0.2/dt * std::max(0.0, penetration - 0.01);
        double j = -1 * (dotProd(relative_vel,normal) );
        j = j/ inverse_mass;
       // j /= contact_num;
        //  if(std::abs(j) < 0.00001f ) return;


        double pn0 = contacts[i].Pn;
        contacts[i].Pn = std::max(pn0 + j, 0.0);
        j = contacts[i].Pn - pn0;

        Vec impulse = j * normal;
        a.apply_impulse(-1 * impulse,ra);
        b.apply_impulse(impulse,rb);
    //    a.contacts.push_back(m.contacts[i]);

         //relative_vel = b.get_velocity()  - a.get_velocity() ;

       // Vec tangent = Vec{normal.get_y()*-1, normal.get_x()};
        //  if(dotProd(b.get_velocity() + cross(rb, b.angular_vel),tangent) > 0) {
        //   tangent = -1*tangent;
        // }

        relative_vel = b.get_velocity() + cross(rb,b.angular_vel) - a.get_velocity() -  cross(ra, a.angular_vel);
        Vec tangent =   cross(normal, 1.0);
        tangent.normalize();

        inverse_mass = a.inv_mass + b.inv_mass + pow(cross(ra, tangent) , 2)*a.inv_inertia + pow(cross(rb, tangent) , 2)*b.inv_inertia;
        double jtt = -dotProd(relative_vel, tangent);
        double jt = jtt / inverse_mass;
        //jt /= contact_num;

      //   if(std::abs(jt) < 0.0001f ) return;
        double static_coefficient  = std::sqrt(a.static_coeff*b.static_coeff);
        double dynamic_coefficient  = std::sqrt(a.dynamic_coeff*b.dynamic_coeff);

        double maxPt = dynamic_coefficient * contacts[i].Pn;
        double oldtimp = contacts[i].Pt;
        contacts[i].Pt = std::max(-maxPt, std::min(oldtimp + jt, maxPt));
        jt = contacts[i].Pt - oldtimp;


        Vec friction_force = jt * tangent;



      b.apply_impulse(friction_force,rb);
      a.apply_impulse(-1 * friction_force,ra);







    }




}

