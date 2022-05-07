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

class DrawBody : public sf::Drawable{
public:
   DrawBody(Body& bod, float height) : body(bod), h(height){}

   void draw(sf::RenderTarget &target, sf::RenderStates states) const override{
       sf::CircleShape circ{body.get_radius()};
       circ.setOrigin(circ.getRadius(),circ.getRadius()*2);
       circ.setFillColor(sf::Color::White);
       circ.setPosition(body.get_position().get_x(), h-body.get_position().get_y());
       target.draw(circ,states);
   }
private:
    Body& body;
   float h;

};




int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Simple Physics Engine");
    float h = sf::VideoMode::getDesktopMode().height;
    float w = sf::VideoMode::getDesktopMode().width;
    Circle circ{100, Vect{w/2,h/2}};
    Body body{circ, 0.4, 2, Vect{1000, 1000}};

    sf::CircleShape shape{circ.get_radius()};
    shape.setFillColor(sf::Color::Black);
    shape.setPosition(circ.get_centre().get_x(), circ.get_centre().get_y());
    float fps = 60;
    float dt = 1/fps;
    sf::Clock clock;
    float accum = 0;
    Vect prev = Vect{body.get_position().get_x(), body.get_position().get_y()};
    Vect curr = Vect{body.get_position().get_x(), body.get_position().get_y()};

    DrawBody bod(body,h);

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
            accum-=dt;
        }

        float a = accum/dt;
        Vect temp{prev.get_x()*a + curr.get_x()*(1-a),prev.get_y()*a + curr.get_y()*(1-a)};
        prev = Vect{body.get_position().get_x(), body.get_position().get_y()};
        curr = temp;
        body.set_position(temp);


        window.draw(bod);
        window.display();







    }
    return 0;
}