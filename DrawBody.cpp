//
// Created by julia on 08/05/22.
//

#include "DrawBody.h"
void DrawBody::draw(sf::RenderTarget &target, sf::RenderStates states) const {
    if(body.get_shape().get_type() == Type::Circle){
        auto rad = (float) body.get_shape().get_radius();
        sf::CircleShape circ{rad};
        circ.setOrigin(circ.getRadius(),circ.getRadius());
        circ.setFillColor(sf::Color::White);
        sf::RectangleShape line(sf::Vector2f(circ.getRadius(), 2));
        line.setOrigin(0,1);

        line.rotate(-90);
        line.rotate(body.angle);
        line.setFillColor(sf::Color::Black);
        line.setPosition(body.get_position().get_x(), h-body.get_position().get_y());
        std::cout<<h<<"\n";
        circ.setPosition((float) body.get_render().get_x(),(float) (h-body.get_render().get_y()));
        circ.rotate(body.angle);
        circ.setOutlineThickness(-2.f);
        circ.setOutlineColor(sf::Color::Black);
      //  std::cout<<"Render: " <<body.get_render().get_x() << " " << body.get_render().get_y() <<"\n";
      if(body.gravity == 0) circ.setFillColor(sf::Color::Red);
        target.draw(circ,states);
        target.draw(line,states);
    }
    if(body.get_shape().get_type() == Type::AABB){
        sf::VertexArray quad(sf::Quads, 4);
        float width = ( float) body.get_shape().get_max().get_x()-body.get_shape().get_min().get_x();
        float height = (float) body.get_shape().get_max().get_y()-body.get_shape().get_min().get_y();
        sf::RectangleShape rectangle(sf::Vector2f(width,height));
       rectangle.setOrigin(width,0);
        rectangle.setPosition(body.get_shape().get_max().get_x(),h-body.get_shape().get_max().get_y());
       // rectangle.setOrigin(width/2,height/2);
      //  rectangle.rotate(body.angle);

        quad[0].position = sf::Vector2f((float)body.get_render().get_x(),h - (float)body.get_render().get_y());
        quad[1].position = sf::Vector2f(quad[0].position.x,(quad[0].position.y+height));
        quad[2].position = sf::Vector2f(quad[1].position.x - width,quad[1].position.y);
        quad[3].position = sf::Vector2f(quad[2].position.x  ,( quad[2].position.y-height));
        rectangle.setFillColor(sf::Color::White);
        if(body.intersecting) rectangle.setFillColor(sf::Color::Red);
        rectangle.setOutlineThickness(-3.f);
        rectangle.setOutlineColor(sf::Color::Black);
        if(body.intersecting) {
            quad[0].color = sf::Color::Red;
            quad[1].color = sf::Color::Red;
            quad[2].color = sf::Color::Red;
            quad[3].color = sf::Color::Red;
        }else{
            quad[0].color = sf::Color::Black;
            quad[1].color = sf::Color::Black;
            quad[2].color = sf::Color::Black;
            quad[3].color = sf::Color::Black;
        }
        Rectangle bound = body.get_shape().get_bounding_box();
        double heightt = bound.max.get_y() - bound.min.get_y();
        double widtht = bound.max.get_x() - bound.min.get_x();
        sf::VertexArray quadd(sf::Quads, 4);
        quadd[0].position = sf::Vector2f(bound.max.get_x(), h - (float)bound.max.get_y());
        quadd[1].position = sf::Vector2f(quadd[0].position.x,(quadd[0].position.y+heightt));
        quadd[2].position = sf::Vector2f(quadd[1].position.x - widtht,quadd[1].position.y);
        quadd[3].position = sf::Vector2f(quadd[2].position.x  ,( quadd[2].position.y-heightt));


        quadd[0].color = sf::Color::Green;
        quadd[1].color = sf::Color::Green;
        quadd[2].color = sf::Color::Green;
        quadd[3].color = sf::Color::Green;
        target.draw(quadd,states);


    /*    std::cout <<"quad: " << quad[0].position.x << " " << quad[0].position.y <<"\n";
        std::cout <<"quad: " << quad[1].position.x << " " << quad[1].position.y <<"\n";
        std::cout <<"quad: " << quad[2].position.x << " " << quad[2].position.y <<"\n";
        std::cout <<"quad: " << quad[3].position.x << " " << quad[3].position.y <<"\n";
        std::cout<<"Render " <<body.get_render().get_x() << " " << body.get_render().get_y() << "\n";
        std::cout<< "pos: " << body.get_position().get_x() << " " << body.get_position().get_y() << "\n";*/
        target.draw(rectangle,states);


    }
    if(body.get_shape().get_type() == Type::OBB){

        OBB ob{body.get_shape().get_max().get_size(), body.get_shape().get_min().get_size(), body.get_shape().get_position()};

        sf::VertexArray quad(sf::Quads, 4);
        Helper_Rect rect = body.get_shape().get_points();
       quad[0].position = sf::Vector2f( rect[0].get_x(), h-rect[0].get_y());
        quad[1].position = sf::Vector2f( rect[1].get_x(), h-rect[1].get_y());
        quad[2].position = sf::Vector2f( rect[2].get_x(), h-rect[2].get_y());
        quad[3].position = sf::Vector2f( rect[3].get_x(), h-rect[3].get_y());
        if(body.intersecting) {
            quad[0].color = sf::Color::Red;
            quad[1].color = sf::Color::Red;
            quad[2].color = sf::Color::Red;
            quad[3].color = sf::Color::Red;
        }else{
            quad[0].color = sf::Color::Black;
            quad[1].color = sf::Color::Black;
            quad[2].color = sf::Color::Black;
            quad[3].color = sf::Color::Black;
        }

        Rectangle bound = body.get_shape().get_bounding_box();
        double height = bound.max.get_y() - bound.min.get_y();
        double width = bound.max.get_x() - bound.min.get_x();
        sf::VertexArray quadd(sf::Quads, 4);
        quadd[0].position = sf::Vector2f(bound.max.get_x(), h - (float)bound.max.get_y());
        quadd[1].position = sf::Vector2f(quadd[0].position.x,(quadd[0].position.y+height));
        quadd[2].position = sf::Vector2f(quadd[1].position.x - width,quadd[1].position.y);
        quadd[3].position = sf::Vector2f(quadd[2].position.x  ,( quadd[2].position.y-height));


        quadd[0].color = sf::Color::Cyan;
        quadd[1].color = sf::Color::Cyan;
        quadd[2].color = sf::Color::Cyan;
        quadd[3].color = sf::Color::Cyan;
        target.draw(quadd,states);
        target.draw(quad,states);

    }

}