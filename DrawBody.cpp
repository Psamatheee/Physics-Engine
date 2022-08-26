//
// Created by julia on 08/05/22.
//

#include "DrawBody.h"

void DrawBody::draw(sf::RenderTarget &target, sf::RenderStates states) const {
    if (body.get_shape().get_type() == Type::Circle) {
        //circle
        auto rad = (float) body.get_shape().get_radius();
        sf::CircleShape circ{rad};
        circ.setOrigin(circ.getRadius(), circ.getRadius());
        circ.setFillColor(sf::Color::White);
        circ.setPosition((float) body.get_render().get_x(), (float) (h - body.get_render().get_y()));
        circ.rotate(body.angle);
        circ.setOutlineThickness(-2.f);
        circ.setOutlineColor(sf::Color::Black);

        //line that shows circle's rotation
        sf::RectangleShape line(sf::Vector2f(circ.getRadius(), 2));
        line.setOrigin(0, 1);
        line.rotate(-90);
        line.rotate(180 / M_PI * body.angle);
        line.setFillColor(sf::Color::Black);
        line.setPosition(body.get_position().get_x(), h - body.get_position().get_y());

        target.draw(circ, states);
        target.draw(line, states);
    }
    if (body.get_shape().get_type() == Type::AABB) {

        sf::VertexArray quad(sf::Quads, 4);
        float width = (float) body.get_shape().get_max().get_x() - body.get_shape().get_min().get_x();
        float height = (float) body.get_shape().get_max().get_y() - body.get_shape().get_min().get_y();
        sf::RectangleShape rectangle(sf::Vector2f(width, height));
        rectangle.setOrigin(width, 0);
        rectangle.setPosition(body.get_shape().get_max().get_x(), h - body.get_shape().get_max().get_y());

        rectangle.setFillColor(sf::Color::White);
        rectangle.setOutlineThickness(-3.f);
        rectangle.setOutlineColor(sf::Color::Black);

        target.draw(rectangle, states);
    }

    if (body.get_shape().get_type() == Type::OBB) {

        sf::VertexArray quad(sf::Quads, 4);
        Helper_Rect rect = body.get_shape().get_points();
        quad[0].position = sf::Vector2f(rect[0].get_x(), h - rect[0].get_y());
        quad[1].position = sf::Vector2f(rect[1].get_x(), h - rect[1].get_y());
        quad[2].position = sf::Vector2f(rect[2].get_x(), h - rect[2].get_y());
        quad[3].position = sf::Vector2f(rect[3].get_x(), h - rect[3].get_y());

        quad[0].color = sf::Color::Black;
        quad[1].color = sf::Color::Black;
        quad[2].color = sf::Color::Black;
        quad[3].color = sf::Color::Black;

        target.draw(quad, states);

    }

}