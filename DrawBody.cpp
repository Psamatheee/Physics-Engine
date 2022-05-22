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
        std::cout<<h<<"\n";
        circ.setPosition((float) body.get_render().get_x(),(float) (h-body.get_render().get_y()));
      //  std::cout<<"Render: " <<body.get_render().get_x() << " " << body.get_render().get_y() <<"\n";
        target.draw(circ,states);
    }
    if(body.get_shape().get_type() == Type::AABB){
        sf::VertexArray quad(sf::Quads, 4);
        float width = ( float) body.get_shape().get_max().get_x()-body.get_shape().get_min().get_x();
        float height = (float) body.get_shape().get_max().get_y()-body.get_shape().get_min().get_y();
        quad[0].position = sf::Vector2f((float)body.get_render().get_x(),h - (float)body.get_render().get_y());
        quad[1].position = sf::Vector2f(quad[0].position.x,(quad[0].position.y+height));
        quad[2].position = sf::Vector2f(quad[1].position.x - width,quad[1].position.y);
        quad[3].position = sf::Vector2f(quad[2].position.x  ,( quad[2].position.y-height));


        quad[0].color = sf::Color::White;
        quad[1].color = sf::Color::White;
        quad[2].color = sf::Color::White;
        quad[3].color = sf::Color::White;
    /*    std::cout <<"quad: " << quad[0].position.x << " " << quad[0].position.y <<"\n";
        std::cout <<"quad: " << quad[1].position.x << " " << quad[1].position.y <<"\n";
        std::cout <<"quad: " << quad[2].position.x << " " << quad[2].position.y <<"\n";
        std::cout <<"quad: " << quad[3].position.x << " " << quad[3].position.y <<"\n";
        std::cout<<"Render " <<body.get_render().get_x() << " " << body.get_render().get_y() << "\n";
        std::cout<< "pos: " << body.get_position().get_x() << " " << body.get_position().get_y() << "\n";*/
        target.draw(quad,states);


    }

}