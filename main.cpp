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
      //  std::cout<<bodies.size()<<"\n";
        for(Body* body : bodies){
            body->integrate(dt,w,h);
            body->set_current(body->get_position());

        }
      //  std::cout<<"eeeeeeeee\n";
        phase.generate_pairs();
        std::vector<Pair> pairs  = phase.get_pairs();
        //std::cout<<"size\n";
        //std::cout<<pairs.size()<<"\n";
        for(Pair pair : pairs){
            if(does_intersect(*pair.a,*pair.b)){
                Body& aa = *pair.a;
                Body& bb = *pair.b;

                Body& a = *pair.a;
                Body& b = *pair.b;
                double e = std::min(a.get_e(), b.get_e());
                Vec a_pos =a.get_position();
                Vec b_pos =b.get_position();
                Vec collision_normal = b_pos - a_pos;
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
          //  std::cout<<"render render render\n";
          //  std::cout<<body->get_render().get_x()<<"\n";
          //  std::cout<<body->get_render().get_y()<<"\n";
        }
       // std::cout << "ee\n";
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
    std::vector<DrawBody> bodies;
};






int main() {
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Simple Physics Engine");
    double h = window.getSize().y;
    double w = window.getSize().x;
    Circle circ{100, Vec{w / 2, h / 2}};
    Circle circ2{150, Vec{500, 500}};
    Circle circ4{200, Vec{200, 200}};
    Circle circ3{50, Vec{(w/3)*2, (h/3)*2}};
    AABB ab2{Vec{400,200}, Vec{550,350}};
    Body bod3{ab2, 0.75, 0.5,Vec{600,600}};
   // Body body2{circ2, 0.75, 3, Vec{100, 500}};
   // Body body{circ, 0.75, 1, Vec{300, 300}};

    AABB ab{Vec{600,600}, Vec{700,700}};
    Body bod4{ab, 0.75, 4.5,Vec{50,60}};

    State state{w,h};
    state.add_body(&bod3);
    //state.add_body(&body);
    //state.add_body(&body2);
    state.add_body(&bod4);
    DrawBodies draw_bodies{};
    draw_bodies.update(state);

    double fps = 60;
    double dt = 1 / fps;
    sf::Clock clock;
    double accum = 0;
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

            accum -= dt;

        }

        double a = accum / dt;
        state.set_render_pos(a);

        //draw_bodies.draw_state(state,window);
         for(const DrawBody& bodd : draw_bodies.bodies){
            window.draw(bodd);
        }
        window.display();
    }









    return 0;
}