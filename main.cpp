#include <iostream>
#include <cmath>

struct point{
    double x;
    double y;
};


class Circle{
public:
    Circle(double r, point c) : (radius = r){}
    double get_radius() const{return radius;}
    point& get_centre(){return centre;}
private:
    double radius;
    point& centre;
};


bool does_circle_intersect(Circle& c1, Circle& c2){
    double distance_sqr = pow(c1.get_centre().x - c2.get_centre().x,2) + pow(c1.get_centre().y - c2.get_centre().y,2);
    return pow(c1.get_radius() + c2.get_radius(),2) >= distance_sqr;
}



int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
