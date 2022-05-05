#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <vector>
#include "Body.cpp"

struct point{
    float x;
    float y;
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

    Circle circ{100, Vect{100,100}};
    Body body{circ, 0.4, 2, Vect{100,60}};

    sf::CircleShape shape{circ.get_radius()};
    shape.setFillColor(sf::Color::Black);
    shape.setPosition(circ.get_centre().get_x(), circ.get_centre().get_y());
    float fps = 60;
    float dt = 1/fps;
    sf::Clock clock;
    float accum = 0;
    Vect prev = circ.get_centre();
    Vect curr = circ.get_centre();

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
            body.integrate(dt);
            accum-=dt;
        }

        float a = accum/dt;
        Vect temp{prev.get_x()*a + curr.get_x()*(1-a),prev.get_y()*a + curr.get_y()*(1-a)};
        prev = body.get_circle().get_centre();
        curr = temp;
        shape.setPosition(curr.get_x(), curr.get_y());
        std::cout << shape.getPosition().x << " " << shape.getPosition().y << "\n";
        window.draw(shape);
        window.display();







    }
    return 0;
}