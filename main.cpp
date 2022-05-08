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
       circ.setOrigin(circ.getRadius(),circ.getRadius());
       circ.setFillColor(color);
       circ.setPosition(body.get_position().get_x(), h-body.get_position().get_y());
       target.draw(circ,states);
   }

    sf::Color color;
private:
    Body& body;
   float h;


};






int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Simple Physics Engine");
    float h = window.getSize().y;
    float w =window.getSize().x;
    Circle circ{100, Vect{w/2,h/2}};
    Circle circ2{150, Vect{500,500}};
    Body body2{circ2, 0.4, 3, Vect{500, 500}};
    Body body{circ, 0.4, 2, Vect{300, 300}};
    Vect centreLine = body2.get_position() - body.get_position();
    centreLine.normalize();
    Manifold m{body,body2,0, centreLine};

    float fps = 60;
    float dt = 1/fps;
    sf::Clock clock;
    float accum = 0;
    Vect prev = Vect{body.get_position().get_x(), body.get_position().get_y()};
    Vect curr = prev;
    Vect prev1 = Vect{body2.get_position().get_x(), body2.get_position().get_y()};
    Vect curr1 = prev1;
    DrawBody bod(body,h);
    DrawBody bod2(body2,h);
    bod.color = sf::Color::Black;
    bod2.color = sf::Color::White;
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
               position_correction(body,body2,m);

            }
            accum-=dt;
        }

        float a = accum/dt;
        curr = Vect{body.get_position().get_x(), body.get_position().get_y()};
        Vect render_pos{prev.get_x()*a + curr.get_x()*(1-a),prev.get_y()*a + curr.get_y()*(1-a)};
        body.set_position(render_pos);

        curr1 = Vect{body2.get_position().get_x(), body2.get_position().get_y()};
        Vect render_pos2{prev1.get_x()*a + curr1.get_x()*(1-a),prev1.get_y()*a + curr1.get_y()*(1-a)};
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