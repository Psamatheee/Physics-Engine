//
// Created by julia on 05/05/22.
//

#ifndef ENGINE_BODY_H
#define ENGINE_BODY_H
#include <cmath>
#include "Geometry.cpp"



class Body{
public:
    Body(Shape& shape, double m, Vec vect) : shape(shape), mass{m}{
        if(mass == 0){
            inv_mass = 0;
            inertia = 0;
            inv_inertia = 0;
        }else{
            inv_mass = 1/mass;
            if(shape.get_type() == Type::OBB){
                inertia = 1.0/12 * mass * (pow(shape.get_max().get_size() * 2, 2) + pow(shape.get_min().get_size() * 2, 2));
                inv_inertia = 1/inertia;
            }else if(shape.get_type() == Type::Circle){
                inertia = 1.0/2 * mass * shape.get_radius()*shape.get_radius();
                inv_inertia = 1/inertia;
            }else{

                inv_inertia = 0;
                inertia = 0;
            }
        }

        velocity.set_x(vect.get_x());
        velocity.set_y(vect.get_y());
        current = shape.get_position();
        previous = shape.get_position();
        render_pos = shape.get_position();
        if(mass == 0) {

            gravity = 0;
        }else{
            gravity = 400;
        }
        angular_vel = 0;
        angle = 0;
        edge.point1 = Vec{};
        edge.point2 = Vec{};
        rest_const = 0.75;

    };

    void apply_impulse(Vec impulse, Vec normal_vec);
    //~Body();

    //getters
    double get_e() const{return rest_const;}
    double get_mass() const{return mass;}
    Vec& get_velocity(){return velocity;}
    Vec get_position()const {return shape.get_position();}
    double get_inv_mass() const{return inv_mass;}
    Shape& get_shape() const {return shape;}
    Vec get_curr(){return current;}
    Vec get_prev(){return previous;}
    Vec get_render()const {return render_pos;}

    //setters
    void set_velocity(double x, double y){
        velocity.set_x(x);
        velocity.set_y(y);
    }
    void set_velocity(Vec vel){
        velocity.set_x(vel.get_x());
        velocity.set_y(vel.get_y());
    }
    void set_position(Vec v){
        shape.set_position(v.get_x(),v.get_y());
    }
    void set_position(double xx, double yy){
        shape.set_position(xx,yy);
    }
    void set_current(Vec vec){
        current = vec;
    }
    void set_previous(Vec vec){
        previous = vec;
    }
    void set_render(Vec vec){
        render_pos = vec;
    }


    void integrate(double dt, double w, double h);

    friend bool operator==(Body a, Body b);
    friend bool operator!=(Body a, Body b);

    double  gravity;
    double mass;
    double inv_mass;
    double dynamic_coeff = 0.28;
    double static_coeff = 0.14;
    double angular_vel;
    double angle;
    double inertia;
    double inv_inertia;
    bool intersecting = false;
    Edge edge;
    std::vector<Vec> contacts;

private:
    Shape& shape;
    double rest_const;

    Vec velocity;
    Vec current;
    Vec previous;
    Vec render_pos;


};


#endif //ENGINE_BODY_H
