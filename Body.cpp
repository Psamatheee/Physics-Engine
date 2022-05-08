//
// Created by julia on 05/05/22.
//

#include "Body.h"
//#include "Geometry.h"
#include <algorithm>



void Body::integrate(float dt, float w, float h) {
    Vec dr{dt * velocity.get_x(), dt * velocity.get_y()};
    float new_x = get_position().get_x() + dr.get_x();
    float new_y = get_position().get_y() + dr.get_y();
    set_position(new_x , new_y);
    collides_wall(h,w);


}

bool does_intersect(Body& a, Body& b){
    return a.shape.intersects(b.shape);
};


bool Body::collides_wall(float h, float w){
    float pen;
    Vec normal{};
    Boundary bound = get_shape().collides_boundary(w,h);
    if(bound == Boundary::None) return false;
    switch (bound) {
        case Boundary::Top:
            set_velocity(get_velocity().get_x(), get_velocity().get_y() * -1);
            pen = get_position().get_y() + get_shape().get_radius() - h;
            normal.set_x(0);
            normal.set_y(1);
            break;
        case Boundary::Bottom:
            set_velocity(get_velocity().get_x(), get_velocity().get_y() * -1);
            pen = -get_position().get_y()+ get_shape().get_radius();
            normal.set_x(0);
            normal.set_y(-1);
            break;
        case Boundary::Right:
            set_velocity(get_velocity().get_x() * -1, get_velocity().get_y() );
            pen = get_position().get_x()+ get_shape().get_radius() - w;
            normal.set_x(1);
            normal.set_y(0);
            break;
        case Boundary::Left:
            set_velocity(get_velocity().get_x() * -1, get_velocity().get_y() );
            pen = -get_position().get_x()+ get_shape().get_radius();
            normal.set_x(-1);
            normal.set_y(0);
            break;

    }

        const float slop = 0.01; // usually 0.01 to 0.1
        const float percent = 0.2; // usually 20% to 80%

        Vec correction = (std::max(pen - slop, 0.0f) / (get_inv_mass() ) * percent) * normal ;
        Vec a_pos = get_position();
        Vec a_correct = get_inv_mass() * correction;
        Vec new_a = a_pos - a_correct ;
        set_position(new_a);




    return true;
}

struct Manifold{
    Body& a;
    Body& b;
    float penetration;
    Vec normal;

};


void position_correction(Body& a, Body& b, Manifold& m){
    const float slop = 0.01; // usually 0.01 to 0.1
    const float percent = 0.2; // usually 20% to 80%
    Vec correction = (std::max(m.penetration - slop, 0.0f) / (a.get_inv_mass() + b.get_inv_mass()) * percent) * m.normal;
    Vec a_pos = a.get_position();
    Vec a_correct = a.get_inv_mass() * correction;
    Vec new_a = a_pos - a_correct ;
    a.set_position(new_a);


    Vec b_pos = b.get_position();
    Vec b_correct = b.get_inv_mass() * correction;
    Vec new_b = b_correct + b_pos;
    b.set_position(new_b);
}


void set_new_speeds(Body& a, Body& b, Manifold& m ){
    Vec a_pos = a.get_position();
    Vec b_pos = b.get_position();
    Vec centreLine = b_pos-a_pos;
    centreLine.normalize();
    m.penetration = get_depth(b.get_shape(), a.get_shape());
    m.normal = centreLine;
    Vec relat_velocity = b.get_velocity() - a.get_velocity();
    if(dotProd(relat_velocity, centreLine)>0) return;
    /*
    std::cout<< "normal line: " << centreLine.get_x() <<  " " << centreLine.get_y() << "\n";
    std::cout<< "a pos " << a.get_position().get_x() << " " << a.get_position().get_y() << "\n";
    std::cout<< "b pos " << b.get_position().get_x() << " " << b.get_position().get_y() << "\n";
    std::cout << "prev  v_x a: " << a.get_velocity().get_x()<<"\n";
    std::cout << "prev  v_x b: " << b.get_velocity().get_x()<<"\n";
    std::cout << "prev  v_y a: " << a.get_velocity().get_y()<<"\n";
    std::cout << "prev  v_y b: " << b.get_velocity().get_y()<<"\n";
*/
    //these are the initial velocity compnenets along the centreline of the 2 circles
    float initial_speed_a = dotProd(a.get_velocity(),centreLine);
    float initial_speed_b = dotProd(b.get_velocity(),centreLine);
 //   std::cout<< "init normal speed a " << initial_speed_a << "\n";
  //  std::cout<< "init normal speed b " << initial_speed_b << "\n";

    float e = std::min(a.get_e(), b.get_e());
    //along the collision normal, the problem becomes 1D
    float final_speed_b = (-e*(initial_speed_b - initial_speed_a) + a.get_mass()*initial_speed_a + b.get_mass()*initial_speed_b) * 1/(1 + b.get_mass());
    float final_speed_a = e*(initial_speed_b - initial_speed_a) + final_speed_b;

    //get full velocity by adding to the full velocity vector
  //if(final_speed_a < 0) final_speed_a= final_speed_a*-1;
   //if(final_speed_b < 0) final_speed_b= final_speed_b*-1;
    Vec a_along_n = scalar_mult((-initial_speed_a + final_speed_a), centreLine);
    Vec final_velocity_a = a.get_velocity() + a_along_n;
    Vec b_along_n = scalar_mult(final_speed_b - initial_speed_b  , centreLine);
    Vec final_velocity_b = b.get_velocity() + b_along_n;
   // std::cout << "final along n a " << a_along_n.get_x() << " " << a_along_n.get_y() <<"\n";
   // std::cout << "final along n b " << b_along_n.get_x() << " " << b_along_n.get_y() <<"\n";

   a.set_velocity(final_velocity_a.get_x(),final_velocity_a.get_y());
    b.set_velocity(final_velocity_b.get_x(),final_velocity_b.get_y());
   // std::cout << "final  v_x a: " << a.get_velocity().get_x()<<"\n";
    //std::cout << "final  v_x b: " << b.get_velocity().get_x()<<"\n";
    //std::cout << "final  v_y a: " << a.get_velocity().get_y()<<"\n";
    //std::cout << "final  v_y b: " << b.get_velocity().get_y()<<"\n";


}


