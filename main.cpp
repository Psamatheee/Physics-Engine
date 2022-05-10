#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <vector>
//#include "Body.cpp"
#include "DrawBody.cpp"






class State{
public:
    State(double ww, double hh) : w(ww), h(hh), phase(BroadPhase(bodies)){
    };
    void add_body(Body* body){bodies.push_back(body);}
    void update_physics(double dt){
        for(Body* body : bodies){
            body->integrate(dt,w,h);
            body->set_current(body->get_position());

        }
        phase.generate_pairs();
        std::vector<Pair> pairs  = phase.get_pairs();
        for(Pair pair : pairs){
            if(does_intersect(*pair.a,*pair.b)){
                Body& aa = *pair.a;
                Body& bb = *pair.b;

                Body& a = *pair.a;
                Body& b = *pair.b;
                double e = std::min(a.get_e(), b.get_e());
                Vec collision_normal = b.get_position() = a.get_position();
                collision_normal.normalize();
                Manifold m = Manifold{a,b,e,collision_normal };
                set_new_speeds(aa,bb,m);
                position_correction(aa,bb,m);

            }

        }



    }

    void set_render_pos(double a){
        for(Body* body : bodies){
            Vec render_pos{body->get_prev().get_x() * a + body->get_curr().get_x() * (1 - a), body->get_prev().get_y() * a + body->get_curr().get_y() * (1 - a)};
            body->set_render(render_pos);
            body->set_previous(body->get_curr());
        }
    }

    std::vector<Body*> get_bodies() const {return bodies;}
    double get_h(){return h;}

private:
    double w;
    double h;
    BroadPhase phase;
    std::vector<Body*> bodies;
};

struct DrawBodies{


    std::vector<DrawBody> bodies;
    void update(State& state){
        std::vector<Body*> state_bodies = state.get_bodies();
        for (Body* body : state_bodies){
            Body& bod = *body;
            DrawBody draw_body{bod, state.get_h(), sf::Color::White};
            bodies.push_back(draw_body);
        }
    }
    void draw_state(State& state, sf::RenderWindow& window){
        for(DrawBody& body : bodies){
            window.draw(body);

        }

    }
};






int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Simple Physics Engine");
    double h = window.getSize().y;
    double w = window.getSize().x;
    Circle circ{100, Vec{w / 2, h / 2}};
    Circle circ2{150, Vec{500, 500}};
    Body body2{circ2, 0.75, 3, Vec{1000, 500}};
    Body body{circ, 0.75, 1, Vec{300, 300}};

    State state{h, w};
    state.add_body(&body);
    state.add_body(&body2);
    DrawBodies draw_bodies{};
    draw_bodies.update(state);
    Circle circe = Circle{30,Vec{20,10}};
    Circle* ee = &circe;
    Circle& ff = *ee;
    std::cout<<ff.get_position().get_y()<<"\n";
    circe.set_position(30,20);
    std::cout<<ff.get_position().get_y()<<"\n";
    std::cout<<circe.get_position().get_y()<<"\n";


    double fps = 60;
    double dt = 1 / fps;
    sf::Clock clock;
    double accum = 0;
    DrawBody bod(body, h, sf::Color::White);
    DrawBody bod2(body2, h, sf::Color::White);
//bool started_resolution = false;
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }

        window.clear(sf::Color::Magenta);
        sf::Time frame_t = clock.restart();
        accum += frame_t.asSeconds();

        if (accum > 0.2) {
            accum = 0.2;
        }

        while (accum > dt) {
            state.update_physics(dt);

        }
        accum -= dt;

        double a = accum / dt;
        state.set_render_pos(a);

        draw_bodies.draw_state(state,window);
        window.display();
    }









    return 0;
}