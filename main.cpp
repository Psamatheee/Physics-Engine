#include <iostream>
#include <cmath>

struct point{
    double x;
    double y;
};
struct Rectangle{
    point min;
    point max;
};

struct Circle{
    double radius;
    point centre;
};

bool does_rect_intersect(Rectangle& r1, Rectangle& r2){
    // check if there is separation along the x-axis
    if (r1.min.x > r2.max.x || r1.max.x < r2.min.x) return false;
    // check if there is separation along the y-axis
    if (r1.min.y > r2.max.y || r1.max.y < r2.min.y) return false;
    return true;
}

bool does_circle_intersect(Circle& c1, Circle& c2){
    double distance_sqr = pow(c1.centre.x - c2.centre.x,2) + pow(c1.centre.y - c2.centre.y,2);
    return pow(c1.radius + c2.radius,2) >= distance_sqr;
}



int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
