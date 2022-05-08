#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <vector>
//#include "Body.cpp"
#include "DrawBody.cpp"

struct point{
    double x;
    double y;
};
struct Rectangle{
    point min;
    point max;
};

bool does_rect_intersect(Rectangle& r1, Rectangle& r2){
    // check if there is separation along the x-axis
    if (r1.min.x > r2.max.x || r1.max.x < r2.min.x) return false;
    // check if there is separation along the y-axis
    if (r1.min.y > r2.max.y || r1.max.y < r2.min.y) return false;
    return true;
}








int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Simple Physics Engine");
    double h = window.getSize().y;
    double w =window.getSize().x;
    Circle circ{100, Vec{w / 2, h / 2}};
    Circle circ2{150, Vec{500, 500}};
    Body body2{circ2, 0.75, 3, Vec{1000, 500}};
    Body body{circ, 0.75, 1, Vec{300, 300}};
    Vec pos = body.get_position();
    Vec pos2 = body2.get_position();
    Vec centreLine = pos2 - pos;
    centreLine.normalize();
    Manifold m{body,body2,0, centreLine};

    double fps = 60;
    double dt = 1/fps;
    sf::Clock clock;
    double accum = 0;
    Vec prev = Vec{body.get_position().get_x(), body.get_position().get_y()};
    Vec curr = prev;
    Vec prev1 = Vec{body2.get_position().get_x(), body2.get_position().get_y()};
    Vec curr1 = prev1;
    DrawBody bod(body,h,sf::Color::White);
    DrawBody bod2(body2,h,sf::Color::White);
//bool started_resolution = false;
    while(window.isOpen()){
        sf::Event event;
        while(window.pollEvent(event)){
            if(event.type == sf::Event::Closed) window.close();
        }

        window.clear(sf::Color::Magenta);
        sf::Time frame_t = clock.restart();
        accum+= frame_t.asSeconds();

        if(accum > 0.2){
            accum = 0.2;
        }
        //std::cout<<accum<<"\n";

        while(accum > dt){
            body.integrate(dt,w, h);
            body2.integrate(dt,w,h);
            if(does_intersect(body,body2) ){
                std::cout << "eee\n";
                set_new_speeds(body,body2,m);
              // position_correction(body,body2,m);

            }
            accum-=dt;
        }

        double a = accum/dt;
        curr = Vec{body.get_position().get_x(), body.get_position().get_y()};
        Vec render_pos{prev.get_x() * a + curr.get_x() * (1 - a), prev.get_y() * a + curr.get_y() * (1 - a)};
        body.set_position(render_pos);

        curr1 = Vec{body2.get_position().get_x(), body2.get_position().get_y()};
        Vec render_pos2{prev1.get_x() * a + curr1.get_x() * (1 - a), prev1.get_y() * a + curr1.get_y() * (1 - a)};
        body2.set_position(render_pos2);



        window.draw(bod);
        window.draw(bod2);
        body.set_position(curr);
        body2.set_position(curr1);
        prev = curr;
        prev1 = curr1;
        window.display();







    }
    return 0;
}