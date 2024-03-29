#include <cmath>
#include <SFML/Graphics.hpp>
#include <vector>
#include "State.h"

/* Clean up to do:
 * change function parameters to body instead of obb so that you dont have to copy it (in progress)
 * change the shape definition of min and max to half width and half height to avoid confusion in obb calculations
 * add collision file to clean up the collision calculations
 * smart pointers
 * clean up algorithms
 * get state to add the bodies
 * make body hold the shape not just the reference
 *
 */


void draw_state(sf::RenderWindow& window, State& state){
    for(Body& body : state.get_bodies()){
        float h = window.getSize().y;
        if (body.shape->get_type() == Type::Circle) {
            //circle
            auto rad = (float) body.shape->get_radius();
            sf::CircleShape circ{rad};
            circ.setOrigin(circ.getRadius(), circ.getRadius());
            circ.setFillColor(sf::Color::Black);
            circ.setPosition((float) body.render_position.x, (float) (h - body.render_position.y));
            circ.rotate(body.angle);
            circ.setOutlineThickness(-3.f);
            circ.setOutlineColor(sf::Color::White);

            //line that shows circle's rotation
            sf::RectangleShape line(sf::Vector2f(circ.getRadius(), 2));
            line.setOrigin(0, 1);
            line.rotate(-90);
            line.rotate(180 / M_PI * body.angle);
            line.setFillColor(sf::Color::White);
            line.setPosition(body.get_position().x, h - body.get_position().y);

            window.draw(circ);
            window.draw(line);
        }

        if (body.shape->get_type() == Type::OBB) {
            double height = 2 *  body.shape->get_max().get_size();
            double width = 2 * body.shape->get_min().get_size();

            sf::VertexArray quad(sf::Quads, 4);
            sf::RectangleShape obb(sf::Vector2f(width,height));
            obb.setOrigin(width/2, height/2);
            obb.move(body.render_position.x, h - body.render_position.y);
            obb.rotate(body.shape->get_orient() *  180/M_PI);
            obb.setFillColor(sf::Color::Black);
            obb.setOutlineColor(sf::Color::White);

            obb.setOutlineThickness(-3.f);
            window.draw(obb);
        }
    }
}


int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Simple Physics Engine");
    double h = window.getSize().y;
    double w = window.getSize().x;

    sf::Font font;
    font.loadFromFile("arial.ttf");
    sf::Text body_number;
    body_number.setFont(font);

    State state{w, h};
    state.reset();

    bool added = false;
    bool added_box = false;

    double fps = 60;
    double dt = 1 / fps;
    sf::Clock clock;
    double accum = 0;
    while (window.isOpen()) {

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
        sf::Vector2i localPosition = sf::Mouse::getPosition(window);

        double x = 0;
        double y = 0;
        double xx = 0;
        double yy = 0;

        while (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            y = h - double(localPosition.y);
            x = double(localPosition.x);
            added = false;

        }
        while (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
            yy = h - double(localPosition.y);
            xx = double(localPosition.x);
            added_box = false;

        }

        if (x != 0 && y != 0 && !added) {
            state.create_body(Type::Circle, Vec{x,y});
            added = true;
        }

        if (xx != 0 && yy != 0 && !added_box) {

            state.create_body(Type::OBB, Vec{xx,yy});
            added_box = true;
        }

        sf::Time frame_t = clock.restart();
        accum += frame_t.asSeconds();

        if (accum > 0.2) {
            accum = 0.2;
        }

        while (accum > dt) {
            state.update_physics(dt );
            accum -= dt;
        }

        double a = accum / dt;
        state.set_render_pos(a);
        window.clear(sf::Color::Black);
      draw_state(window,state);

      body_number.setString("Number of Bodies: " + std::to_string(state.get_size()));
      body_number.setFillColor(sf::Color::White);
      window.draw(body_number);

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::X)) {
            state.reset();
        }

        window.display();
    }

    return 0;
}