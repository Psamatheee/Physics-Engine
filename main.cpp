#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <vector>
//#include "Body.cpp"
//#include "DrawBody.cpp"
#include <string>
#include "State.cpp"

int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Simple Physics Engine");
    double h = window.getSize().y;
    double w = window.getSize().x;

    OBB obbe{Vec{0,50},Vec{500,0}, Vec{w/2,h/4}};
    Body oriented{obbe,0,Vec{0,0}};

    std::vector<Body> bodes;
    std::vector<Circle> shapes;

   State state{w,h};

   int countt = 0;
    oriented.gravity = 0;
oriented.impulse = Vec{0,0};
oriented.angular_vel = 0;
   state.add_body(&oriented);
    std::vector<AABB> boxes;
    std::vector<OBB> orients;
    std::vector<Body*> user_bodies;

 bool added = false;
    bool added_box = false;

    DrawBodies draw_bodies{};
    draw_bodies.update(state);


    double fps = 60;
    double dt = 1 / fps;
    sf::Clock clock;
    double accum = 0;
    int count = 0;
    while (window.isOpen()) {

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                for(Body* bode : user_bodies){
                }
                window.close();}
        }
        sf::Vector2i localPosition = sf::Mouse::getPosition(window);

        double x = 0;
        double y = 0;
        double xx = 0;
        double yy = 0;

        while( sf::Mouse::isButtonPressed(sf::Mouse::Left))
        {
            y = h -double (localPosition.y);
            x = double (localPosition.x);
            added = false;

        }
        while( sf::Mouse::isButtonPressed(sf::Mouse::Right))
        {
            yy = h -double (localPosition.y);
            xx = double (localPosition.x);
            added_box = false;

        }


        if(x != 0 && y !=0 && !added  ){

          auto* circ_testee = new Circle{(double) (std::rand() % 100 + 10), Vec{x,y}};
          Body* testee = new Body{*circ_testee, (double) (std::rand() % 25 + 0.5), Vec{(double) (std::rand() % 1600 + 1 - 800), (double) (std::rand() % 1600 + 1 - 800)    }};
            countt++;
           testee->set_velocity(0,0);
          //  testee->gravity = 500;
           // testee->impulse = Vec{0,0};
            testee->angular_vel = 0;
          state.add_body(testee);
          added = true;

        }

        if(xx != 0 && yy !=0 && !added_box){

            double rand_width = (double) (std::rand() % 200) + 10;
            double rand_height = (double) (std::rand() % 200) + 10;

            auto* aabb_test = new AABB{Vec{xx-rand_width/2, yy-rand_height/2}, Vec{xx + rand_width/2,yy + rand_height/2}};
           // Body* testee = new Body{*aabb_test, 0.75, (double) (std::rand() % 25 + 0.5), Vec{(double) (std::rand() % 1600 + 1 - 800), (double) (std::rand() % 1600 + 1 - 800)    }};
           auto* obb = new OBB{Vec{0,50},Vec{50,0},Vec{xx,yy}};
           Body* testee = new Body{*obb, 3, Vec{} };
          // testee->get_shape().rotate(10 * M_PI * 1/180);
           //testee->set_velocity(0,-70);
          //  testee->mass =( testee->get_shape().get_y() - testee->get_shape().get_min().get_y()) * ( testee->get_shape().get_x() - testee->get_shape().get_min().get_x()) /1000;
          testee->mass = 4;
          //  testee->gravity = 0;
          //  testee->impulse = Vec{0,0};
            testee->angular_vel = 0;
            state.add_body(testee);


            added_box = true;

        }
        for(Body bode : bodes){
            state.add_body(&bode);
        }

        draw_bodies.update(state);



        sf::Time frame_t = clock.restart();
        accum += frame_t.asSeconds();

        if (accum > 0.2) {
            accum = 0.2;
        }

        while (accum > dt) {
            count++;
            state.update_physics(dt, count);

            accum -= dt;

        }

        double a = accum / dt;
        state.set_render_pos(a);
        window.clear(sf::Color::White);
         for(const DrawBody& bodd : draw_bodies.bodies){
            window.draw(bodd);
        }
        window.display();
        if(sf::Keyboard::isKeyPressed(sf::Keyboard::X)){
            state.reset();
        }

        if (count >= state.max_count) count = 0;



    }









    return 0;
}