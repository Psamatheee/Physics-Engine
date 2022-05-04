#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <vector>


class Vect{
public:
    Vect(double xx, double yy) : x{xx}, y{yy}{
        size = sqrt(x*x + y*y);
    }
    Vect() : x{0}, y{0}{
        size = 0;
    }

    double get_x(){return x;}
    double get_y(){return y;}

    void set_x(double xx){x = xx; }
    void set_y(double yy){y = yy;}

    void normalize(){
        x = x/size;
        y = y/size;
    }

    friend Vect operator-(Vect& v1, Vect& v2){
        Vect v{v1.x - v2.x, v1.y - v2.y};
        return v;
    }

    friend Vect operator+(Vect& v1, Vect& v2){
        Vect v{v1.x + v2.x, v1.y + v2.y};
        return v;
    }






private:
    double x;
    double y;
    double size;
};

double dotProd(Vect& v1, Vect& v2){
    return v1.get_x() * v2.get_x() + v1.get_y() * v2.get_y();
}

Vect scalar_mult(double num, Vect& v){
    return {v.get_x()*num, v.get_y()*num};
}


class Circle{
public:
    Circle(double r, Vect c) : radius{r}, centre{c}{}
    double get_radius() const{return radius;}
    Vect& get_centre(){return centre;}


private:
    double radius;
    Vect& centre; //position vector to the centre of the circle
};


bool does_circle_intersect(Circle& c1, Circle& c2){
    //using sqr instead of sqrt as it's more efficient
    double distance_sqr = pow(c1.get_centre().get_x() - c2.get_centre().get_x(),2) + pow(c1.get_centre().get_y() - c2.get_centre().get_y(),2);
    return pow(c1.get_radius() + c2.get_radius(),2) >= distance_sqr;
}

class Body{
public:
    Body(Circle& circ, double restitution_const, double m, Vect vect) : circle{circ}, rest_const{restitution_const}, mass{m}{
        inv_mass= 1/mass;
        velocity.set_x(vect.get_x());
        velocity.set_y(vect.get_y());
    };

    Circle& get_circle(){return circle;}
    double get_e(){return rest_const;}

    double get_mass(){return mass;}
    Vect& get_velocity(){return velocity;}
    void set_velocity(double x, double y){
        velocity.set_x(x);
        velocity.set_y(y);
    }

    void increase_velocity(Vect v){
        velocity = velocity + v;
    }


private:
    Circle& circle;
    double rest_const;
    double mass;
    double inv_mass;
    Vect velocity;


};

struct Manifold{
    Body& a;
    Body& b;
};

void set_new_speeds(Body& a, Body& b ){

    Vect centreLine = b.get_circle().get_centre() - a.get_circle().get_centre();
    centreLine.normalize();

    //these are the initial velocity compnenets along the centreline of the 2 circles
    double initial_speed_a = dotProd(a.get_velocity(),centreLine);
    double initial_speed_b = dotProd(b.get_velocity(),centreLine);

    double e = std::min(a.get_e(), b.get_e());

    double final_speed_b = (-e*(initial_speed_b - initial_speed_a) + a.get_mass()*initial_speed_a + b.get_mass()*initial_speed_b) * 1/(1 + b.get_mass());
    double final_speed_a = e*(initial_speed_b - initial_speed_a) + final_speed_b;

    Vect a_along_n = scalar_mult(final_speed_a,centreLine);
    Vect final_velocity_a = a.get_velocity() +  a_along_n;
    Vect b_along_n = scalar_mult(final_speed_b,centreLine);
    Vect final_velocity_b = b.get_velocity() +  b_along_n;

    a.increase_velocity(final_velocity_a);
    b.increase_velocity(final_velocity_b);
}

bool collision(Manifold& m){
    if (does_circle_intersect(m.a.get_circle(), m.b.get_circle())){
        set_new_speeds(m.a,m.b);
        return true;
    }else{
        return false;
    }
}

struct Object_Set{
    std::vector<Body> bodies;
};


int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Simple Physics Engine");
    Object_Set objects;
    Circle circ{10, Vect{100,100}};
    Body body{circ, 0.4, 2, Vect{2,1}};

    while(window.isOpen()){
        sf::Event event;
        while(window.pollEvent(event)){
            if(event.type == sf::Event::Closed) window.close();
        }




    }
    return 0;
}
