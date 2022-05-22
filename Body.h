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
            gravity = 500;
        }

    };

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


    bool collides_wall(double h, double w,double dt);
    void integrate(double dt, double w, double h);

    friend bool does_intersect(Body& a, Body& b);
    friend bool operator==(Body a, Body b);
    friend bool operator!=(Body a, Body b);
double  gravity;
Vec normal = Vec{};

private:
    Shape& shape;
    double rest_const;
    double mass;
    double inv_mass;
    Vec velocity;
    Vec current;
    Vec previous;
    Vec render_pos;


};


#endif //ENGINE_BODY_H