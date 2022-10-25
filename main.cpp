#include <cmath>
#include <SFML/Graphics.hpp>
#include <vector>
#include "State.h"

int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Simple Physics Engine");
    double h = window.getSize().y;
    double w = window.getSize().x;

    State state{w, h};
    state.reset();
    std::vector<AABB> boxes;
    std::vector<OBB> orients;
    std::vector<Body *> user_bodies;

    bool added = false;
    bool added_box = false;

    DrawBodies draw_bodies{};
    draw_bodies.update(state);


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

            auto *circ_shape = new Circle{(double) (std::rand() % 100 + 10), Vec{x, y}};
            Body *user_circle = new Body{*circ_shape,
                                         1.0 / 2000 * M_PI * circ_shape->get_radius() * circ_shape->get_radius(),
                                       };
            state.add_body(user_circle);
            user_circle->set_velocity(Vec{});
            added = true;
        }

        if (xx != 0 && yy != 0 && !added_box) {
            auto *obb = new OBB{Vec{0, 50}, Vec{50, 0}, Vec{xx, yy}};
            Body *user_obb = new Body{*obb, 3};
            state.add_body(user_obb);
            added_box = true;
        }

        

        draw_bodies.update(state);

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
        window.clear(sf::Color::White);
        for (const DrawBody &draw_body: draw_bodies.bodies) {
            window.draw(draw_body);
        }

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::X)) {
            state.reset();
        }

        window.display();
    }

    return 0;
}