//
// Created by julia on 08/05/22.
//

#ifndef ENGINE_DRAWBODY_H
#define ENGINE_DRAWBODY_H
#include <SFML/Graphics.hpp>
//#include "Geometry.cpp"
//#include "Body.cpp"
#include "Manifold.cpp"


class DrawBody : public sf::Drawable{
public:
    DrawBody(Body& bod, double height, sf::Color color) : body(bod), h(height), color(color){}
    void draw(sf::RenderTarget &target, sf::RenderStates states) const override;

private:
    const Body& body;
    double h;
    sf::Color color;

};


#endif //ENGINE_DRAWBODY_H
