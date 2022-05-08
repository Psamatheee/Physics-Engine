//
// Created by julia on 08/05/22.
//

#include "DrawBody.h"
void DrawBody::draw(sf::RenderTarget &target, sf::RenderStates states) const {
    if(body.get_shape().get_type() == Type::Circle){
        auto rad = (float) body.get_shape().get_radius();
        sf::CircleShape circ{rad};
        circ.setOrigin(circ.getRadius(),circ.getRadius());
        circ.setFillColor(color);
        circ.setPosition((float) body.get_position().get_x(),(float) (h-body.get_position().get_y()));
        target.draw(circ,states);
    }

}