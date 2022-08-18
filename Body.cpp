//
// Created by julia on 05/05/22.
//

#include "Body.h"
//#include "Geometry.h"
#include <algorithm>
#include <iostream>
#include <utility>


void Body::apply_impulse(Vec imp, Vec normal) {
    velocity  = velocity +  inv_mass * imp;
    double crosss = -cross( normal, imp );
    angular_vel += inv_inertia * crosss;
}

void Body::integrate(double dt, double w, double h) {
//    v += (1/m * F) * dt








//angle += angular_vel * dt;

   // set_velocity(ee);



    Vec dr{dt * velocity.get_x(), dt * velocity.get_y()};


    double new_x = get_position().get_x() + dr.get_x();
    double new_y = get_position().get_y() + dr.get_y();
    set_position(new_x , new_y);
    shape.rotate(angular_vel*dt);
    angle = shape.get_orient();


    double ek = 0.5 * mass * velocity.get_size()*velocity.get_size();
    Vec v{velocity.get_x(), velocity.get_y() - dt * gravity*1/2};
    set_velocity(v);


    end_pos = get_position();





}



struct Pair{
    Body* a;
    Body* b;
};



bool operator==(Body a, Body b) {
    bool vecs = (a.get_position().get_x() == b.get_position().get_x() && a.get_position().get_y() == b.get_position().get_y() && a.get_velocity().get_y() == b.get_velocity().get_y() &&a.get_velocity().get_x() == b.get_velocity().get_x()  );
    bool mass = (a.get_mass() == b.get_mass());
    return mass && vecs;
}

bool operator!=(Body a, Body b) {
    return !(a==b);
}


struct Manifold{
    Body& a;
    Body& b;
    double penetration;
    Vec normal;
    std::vector<Vec> contacts;


};


void position_correction(Manifold& m){

    const double slop = 0.07; // usually 0.01 to 0.1
    const double percent = 0.4; // usually 20% to 80%
  //  if(m.a.get_shape().get_type() == Type::OBB || m.b.get_shape().get_type() == Type::OBB) return;
    if(m.penetration != m.penetration){
        int i =0 ;
    }
    Vec correction = (std::max(m.penetration - slop, 0.0) / (m.a.get_inv_mass() + m.b.get_inv_mass()) * percent) * m.normal;
    Vec a_pos = m.a.get_position();
    Vec a_correct = m.a.get_inv_mass() * correction;
    Vec new_a = a_pos - a_correct ;
    m.a.set_position(new_a);


    Vec b_pos = m.b.get_position();
    Vec b_correct = m.b.get_inv_mass() * correction;
    Vec new_b = b_correct + b_pos;
    m.b.set_position(new_b);




}

// a is the rectangle
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
   // if(bb.get_centre().get_x() < aa.max.get_x() && bb.get_centre().get_x() >aa.min.get_x() && bb.get_centre().get_y() > aa.min.get_y() && bb.get_centre().get_y() < aa.max.get_y()){
     //  b.set_position(closest);
     //  centreLine = Vec{centreLine.get_x()*-1, centreLine.get_y()*-1};
//
  //  }

    centreLine.normalize();
    m.normal = Vec{centreLine.get_x(), centreLine.get_y()};
    m.contacts.push_back(closest);
}

int clip(Vec normal, double dot_prod, Edge& edge){

    double d1 = dotProd(normal, edge[0]) - dot_prod;
    double d2 = dotProd(normal, edge[1]) - dot_prod;
    int count = 0;
    if(d1 < 0) count++;
    if(d2 < 0) count++;

    //clip 1st point
    if(d1 * d2 < 0){
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

void set_manifold(Body& a, Body& b, Manifold& m){
    if ((a.get_shape().get_type() == Type::Circle && b.get_shape().get_type() == Type::Circle)) {
        Vec a_position = a.get_position();
        Vec b_position = b.get_position();
        Vec normal = b_position - a_position;
        normal.normalize();
        m.normal = normal;
        m.penetration = get_depth(a.get_shape(),b.get_shape());
        double alpha = std::abs(dotProd(b.get_position(), m.normal) - dotProd(a.get_position(), m.normal )) - b.get_shape().get_radius();
        double x = a.get_shape().get_radius() - alpha;
        Vec contact =  a.get_position()  + x/2  * m.normal;
        m.contacts.push_back(contact);
    }
    if(a.get_shape().get_type() == Type::AABB && b.get_shape().get_type() == Type::Circle){
      //  calculate_manifold_AABBvsCircle(a,b,m);
    }
    if(b.get_shape().get_type() == Type::AABB && a.get_shape().get_type() == Type::Circle){
     //   calculate_manifold_AABBvsCircle(b,a,m);
        m.normal = -1.0 * m.normal ;
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
        calculate_manifold_AABBvsCircle(model_rect, circ, m);
        Vec contact = m.contacts[0];
        cos = std::cos((-angle ));
         sin = std::sin(-angle );
        Matrix matrix2{cos, sin, -sin, cos};
        contact = matrix2 * contact;
        m.contacts[0] = contact;
        m.normal = matrix2 * m.normal;
        return;
    }

    if(b.get_shape().get_type() == Type::OBB && a.get_shape().get_type() == Type::Circle ){
        set_manifold(b,a,m);
        m.normal = -1 * m.normal;
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
                // Find out which axis is axis of least penetration
                if(x_overlap < y_overlap)
                {
                    // Point towards B knowing that n points from A to B
                    if(r.get_x() < 0) m.normal = Vec{-1, 0 };
                    else m.normal = Vec{ 1, 0 };
                    m.penetration = x_overlap;

                }
                else
                {
                    // Point toward B knowing that n points from A to B
                    if(r.get_y() < 0)
                        m.normal = Vec{ 0, -1 };
                    else
                        m.normal = Vec{ 0, 1 };
                    m.penetration = y_overlap;
                }
            }
        }
    }

    if(a.get_shape().get_type() == Type::OBB && b.get_shape().get_type() == Type::OBB ){

        double penetration;
        Vec normal{};
        OBB aa{a.get_shape().get_max(), a.get_shape().get_min(), a.get_position()};
        aa.orient = a.get_shape().get_orient();
        OBB bb{b.get_shape().get_max(), b.get_shape().get_min(), b.get_position()};
        bb.orient = b.get_shape().get_orient();

        Edge edgeA;
        Edge edgeB;
        Edge ref;
        Edge inc;

        double penetration_a = get_collision_normal(aa,bb,edgeA);
        double penetration_b = get_collision_normal(bb,aa,edgeB);

        bool flip;
        if(penetration_a <= penetration_b){
            //a has the reference face
            penetration = penetration_a;
            ref = edgeA;
            inc = edgeB;
            flip = false;
        }
        if(penetration_b < penetration_a){
            //b has the reference face
            ref = edgeB;
            inc = edgeA;
            penetration = penetration_b;
            flip = true;
        }

       Vec temp = ref[1] - ref[0];
        normal.set_x(temp.get_y() * -1);
        normal.set_y(temp.get_x());
        normal.normalize();
        !flip? set_incident_edge(bb,edgeB,normal) : set_incident_edge(aa,edgeA, normal);
        inc = !flip? edgeB : edgeA;

        m.normal = normal;
        m.penetration = penetration;
        a.edge = edgeA;
        b.edge = edgeB;
        //clip the sides of the reference face

        m.normal = normal;
        m.penetration = penetration;

        //clip the sides of the reference face
       Vec side_normal = ref[1] - ref[0];
        side_normal.normalize();
        Vec l = ref[0];
        Vec r = ref[1];
        if(ref[0].get_x() > ref[1].get_x()){
            l = ref[1];
            r = ref[0];
            side_normal = -1*side_normal;
        }
        double left = dotProd(-1 * side_normal, l);
        double right = dotProd( side_normal, r);

        if( clip(side_normal, right, inc) < 2) {
            m.penetration = 0;
            return;
        }
        if( clip(-1 * side_normal, left, inc) < 2) {

            m.penetration = 0;
            return;
        }

        m.normal = normal;
        //pick only points behind reference edge;
       double distance = dotProd(normal, ref[0]);
       double point1_dist = dotProd(inc[0],normal);
        double point2_dist = dotProd(inc[1],normal);
        int count = 0;
       if(point1_dist < distance){
           m.contacts.push_back(inc[0]) ;
           m.penetration = distance - point1_dist;
           count++;
       }else{
           m.penetration = 0;
       }
        if(point2_dist < distance){
            m.contacts.push_back(inc[1]) ;
            m.penetration += distance - point2_dist;
            count++;
        }
        if(count) {

            m.penetration /= count;
        }

        if(flip ) normal = -1 * normal;
        m.normal = normal;
    }
}






void set_new_speeds( Manifold& m, double dt ){

   // m.penetration = 0;
    Body& a  = m.a;
    Body& b  = m.b;
    set_manifold(a, b, m);


    double e = std::min(a.get_e(), b.get_e());



        if(a.sleep) a.sleep = false;
        if(b.sleep) b.sleep = false;
        for(int i = 0; i < m.contacts.size(); i++){
            Vec ra = m.contacts[i] - a.get_position();
            Vec rb = m.contacts[i] - b.get_position();
            Vec relative_vel = b.get_velocity() + cross(rb, b.angular_vel) - a.get_velocity() -  cross(ra, a.angular_vel);
            if(dotProd(relative_vel,m.normal) > 0) return; //moving away from each other
            double inverse_mass = a.inv_mass + b.inv_mass + pow(cross(ra, m.normal) , 2)*a.inv_inertia + pow(cross(rb, m.normal) , 2)*b.inv_inertia;
            double j = -(1.0+e) * (dotProd(relative_vel,m.normal));
            j = j/ inverse_mass;
            j /= m.contacts.size();
            Vec impulse = j * m.normal;
            a.apply_impulse(-1 * impulse,ra);
            b.apply_impulse(impulse,rb);
            a.contacts.push_back(m.contacts[i]);



            Vec tangent = Vec{m.normal.get_y()*-1, m.normal.get_x()};
            if(dotProd(b.get_velocity(),tangent) > 0) {
                tangent = -1*tangent;
            }
            tangent.normalize();
            relative_vel = b.get_velocity() + cross(rb, b.angular_vel) - a.get_velocity() -  cross(ra, a.angular_vel);
            inverse_mass = a.inv_mass + b.inv_mass + pow(cross(ra, tangent) , 2)*a.inv_inertia + pow(cross(rb, tangent) , 2)*b.inv_inertia;
            double jtt = dotProd(relative_vel, tangent);
            double jt = jtt / inverse_mass;

            if(std::abs(jt) < 0.0001f ) return;
            double static_coefficient  = std::sqrt(a.static_coeff*b.static_coeff);
            double dynamic_coefficient  = std::sqrt(a.dynamic_coeff*b.dynamic_coeff);
            Vec friction_force;
            if(std::abs(jt) < impulse.get_size() *  static_coefficient){
                friction_force = std::abs(jt) * tangent;
            }else{
                friction_force = impulse.get_size() * dynamic_coefficient * tangent;
            }


            b.apply_impulse(friction_force,rb);
            a.apply_impulse(-1 * friction_force,ra);







        }
        return;




}





class BroadPhase{
public:
   // explicit BroadPhase(std::vector<Body> bodies) : bodies(std::move(bodies)){}
    explicit BroadPhase(std::vector<Body*>&  bodies) : bodies(bodies){
       pairs.clear();
   }
   void add_body(Body* bod){bodies.push_back(bod);}
    void generate_pairs(){
        pairs.clear();
        Rectangle a;
        Rectangle b;
        for(int i =0; i < bodies.size(); i++){
            int j = i + 1;
            a.min = bodies[i]->get_shape().get_bounding_box().min;
            a.max = bodies[i]->get_shape().get_bounding_box().max;
            while (j < bodies.size()){
                    b.min = bodies[j]->get_shape().get_bounding_box().min  ;
                b.max = bodies[j]->get_shape().get_bounding_box().max  ;
                if(does_rect_intersect(a,b) && !(bodies[i]->sleep && bodies[j]->sleep) ){
                        Body* ap = bodies[i];
                        Body* bp = bodies[j];
                        pairs.push_back(Pair{ap,bp});
                    }
                    j++;
            }
        }


    }

    std::vector<Pair> get_pairs(){return pairs;}
private:
    std::vector<Body*>& bodies;
    std::vector<Pair> pairs;
};


