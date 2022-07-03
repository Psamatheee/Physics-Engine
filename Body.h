//
// Created by julia on 05/05/22.
//

#ifndef ENGINE_BODY_H
#define ENGINE_BODY_H
#include <cmath>
#include "Geometry.cpp"



class Body{
public:
    Body(Shape& shape, double restitution_const, double m, Vec vect) : shape(shape), rest_const{restitution_const}, mass{m}{
        if(mass == 0){
            inv_mass = 0;
        }else{
            inv_mass = 1/mass;
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
        back_up_mass = mass;
        back_up_grav = gravity;
        impulse.set_x(0);
        impulse.set_y(-mass*gravity);
        backup_inv = inv_mass;
        torque = 0;
        angular_vel = 0;
        angle = 0;
        I = 0;




    };
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
    double back_up_mass;
    Vec impulse;
    double inv_mass;
    double backup_inv;
Vec normal = Vec{};
double dynamic_coeff = 0.021;
    double static_coeff = 0.014;
double back_up_grav;
   // OBB orientation;
    double angular_vel;
    double angle;
    bool intersecting = false;
private:
    Shape& shape;
    double rest_const;
    //double mass;

    Vec velocity;
    Vec current;
    Vec previous;
    Vec render_pos;

    double torque;
    double I;


};


#endif //ENGINE_BODY_H
