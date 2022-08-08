//
// Created by julia on 05/05/22.
//

#include "Body.h"
//#include "Geometry.h"
#include <algorithm>
#include <iostream>
#include <utility>


void Body::integrate(double dt, double w, double h) {
//    v += (1/m * F) * dt
if(impulse.get_size() != 0) {
    velocity = velocity + inv_mass * impulse;
}else{
    Vec v{velocity.get_x() ,velocity.get_y() - dt*gravity };
    set_velocity(v);
}
if(I == 0) angular_vel += torque * (0) * dt;
else angular_vel += torque * (1/I) * dt;
//angle += angular_vel * dt;
shape.rotate(angular_vel*dt);
angle = shape.get_orient();
   // set_velocity(ee);



    Vec dr{dt * velocity.get_x(), dt * velocity.get_y()};

    double new_x = get_position().get_x() + dr.get_x();
    double new_y = get_position().get_y() + dr.get_y();
    set_position(new_x , new_y);
    double ek = 0.5 * mass * velocity.get_size()*velocity.get_size();







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

    const double slop = 0.05; // usually 0.01 to 0.1
    const double percent = 0.4; // usually 20% to 80%
    Body a = m.a;
    Body b = m.b;
    if(a.get_shape().get_type() == Type::OBB || b.get_shape().get_type() == Type::OBB) return;

    Vec correction = (std::max(m.penetration - slop, 0.0) / (a.get_inv_mass() + b.get_inv_mass()) * percent) * m.normal;
    Vec a_pos = a.get_position();
    Vec a_correct = a.get_inv_mass() * correction;
    Vec new_a = a_pos - a_correct ;
    a.set_position(new_a);


    Vec b_pos = b.get_position();
    Vec b_correct = b.get_inv_mass() * correction;
    Vec new_b = b_correct + b_pos;
    b.set_position(new_b);




}

// a is the rectangle
void calculate_manifold_AABBvsCircle(Body& a, Body& b, Manifold& m){
    Rectangle aa{a.get_shape().get_min(), a.get_shape().get_max()};
    Circle bb{b.get_shape().get_radius(), b.get_shape().get_position()};
    Vec closest = get_closest_point(aa,bb);
    double distance_sqr = pow(closest.get_x() - bb.get_x(), 2) +
                          pow(closest.get_y() - bb.get_y(), 2);
    double penetration = bb.get_radius() - sqrt(distance_sqr);
    m.penetration = penetration;

    Vec b_pos = b.get_position();
    Vec a_pos = closest;
    Vec centreLine = b_pos - a_pos;
    if(bb.get_centre().get_x() < aa.max.get_x() && bb.get_centre().get_x() >aa.min.get_x() && bb.get_centre().get_y() > aa.min.get_y() && bb.get_centre().get_y() < aa.max.get_y()){
       b.set_position(closest);
       centreLine = Vec{centreLine.get_x()*-1, centreLine.get_y()*-1};

    }

    centreLine.normalize();
    m.normal = Vec{centreLine.get_x(), centreLine.get_y()};
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
            edge[1] = edge[1] - d1/(d1-2) * (edge[0] - edge[1]);
            count++;
        }
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
    }
    if(a.get_shape().get_type() == Type::AABB && b.get_shape().get_type() == Type::Circle){
        calculate_manifold_AABBvsCircle(a,b,m);
    }
    if(b.get_shape().get_type() == Type::AABB && a.get_shape().get_type() == Type::Circle){
        calculate_manifold_AABBvsCircle(b,a,m);
        m.normal = -1.0 * m.normal ;
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
        if(penetration_a < penetration_b){
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

       normal = ref[1] - ref[0];
        normal.set_x(normal.get_y() * -1);
        normal.set_y(normal.get_x());
        normal.normalize();


        m.normal = normal;
        m.penetration = penetration;

        //clip the sides of the reference face

        m.normal = normal;
        m.penetration = penetration;

        //clip the sides of the reference face
       Vec side_normal = ref[1] - ref[0];
        side_normal.normalize();

        double left = dotProd(-1 * side_normal, ref[0]);
        double right = dotProd( side_normal, ref[1]);

        if( clip(side_normal, right, inc) > 2) return;
        if( clip(-1 * side_normal, left, inc) > 2) return;

        if(flip ) normal = -1 * normal;
        //pick only points behind reference edge;
       double distance = dotProd(normal, ref[0]);
       double point1_dist = dotProd(inc[0],normal);
        double point2_dist = dotProd(inc[1],normal);
        int count = 0;
       if(point1_dist < distance){
           m.contacts.push_back(inc[0]) ;
           m.penetration = distance - point1_dist;
           count++;
       }
        if(point2_dist < distance){
            m.contacts.push_back(inc[1]) ;
            m.penetration = distance - point2_dist;
            count++;
        }
        m.penetration/=count;

    }
}






void set_new_speeds(Body& a, Body& b, Manifold& m, double dt ){

    set_manifold(a, b, m);


    if(a.get_shape().get_type() == Type::OBB || b.get_shape().get_type() == Type::OBB) {

    }

    double Uab_normal = dotProd(b.get_velocity()-a.get_velocity(), m.normal); // initial relative velocity along the normal
    if(Uab_normal > 0) return; //moving away from each other
    double e = std::min(a.get_e(), b.get_e());

    //Have to do it this way so that will still work for 0 mass objects
    double j = (-1.0 * (1+e) * Uab_normal) / (a.get_inv_mass() + b.get_inv_mass());
    Vec impulse = j * m.normal;
    a.impulse = a.impulse - impulse;
    b.impulse = b.impulse + impulse;

    Vec tangent = Vec{m.normal.get_y()*-1, m.normal.get_x()};
    if(dotProd(b.get_velocity(),tangent) > 0) {
        tangent = -1*tangent;
    }
    tangent.normalize();
    Vec new_relative = b.get_velocity() - a.get_velocity();
    double jtt = dotProd(new_relative , tangent);
    double jt = jtt / (a.get_inv_mass() + b.get_inv_mass());

    double static_coefficient  = std::sqrt(a.static_coeff*a.static_coeff + b.static_coeff*b.static_coeff);
    double dynamic_coefficient  = std::sqrt(a.dynamic_coeff*a.dynamic_coeff + b.dynamic_coeff*b.dynamic_coeff);
    Vec friction_force;
    if(std::abs(jt) < impulse.get_size() *  static_coefficient){
        friction_force = std::abs(jt) * tangent;
    }else{
        friction_force = impulse.get_size() * dynamic_coefficient * tangent;
    }

a.impulse = a.impulse - friction_force;
    b.impulse = b.impulse + friction_force;



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
                if(does_rect_intersect(a,b)){
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


