//
// Created by julia on 08/05/22.
//

#include "DrawBody.h"
void DrawBody::draw(sf::RenderTarget &target, sf::RenderStates states) const {
    if(body.get_shape().get_type() == Type::Circle){
        sf::CircleShape circ{body.get_shape().get_radius()};
        circ.setOrigin(circ.getRadius(),circ.getRadius());
        circ.setFillColor(color);
        circ.setPosition(body.get_position().get_x(), h-body.get_position().get_y());
        target.draw(circ,states);
    }

}