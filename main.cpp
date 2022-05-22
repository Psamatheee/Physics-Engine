#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <vector>
//#include "Body.cpp"
#include "DrawBody.cpp"
#include <string>






class State{
public:
    State(double ww, double hh) : w(ww), h(hh), phase(BroadPhase(bodies)){
    };
    void add_body(Body* body){
        for(Body* bod : bodies){
            if(bod == body){
                return;
            }
                }
        bodies.push_back(body);


        }


    void update_physics(double dt){

        phase.generate_pairs();
        std::vector<Pair> pairs  = phase.get_pairs();
        for(Pair pair : pairs){
            if(pair.a->get_shape().intersects(pair.b->get_shape()) || pair.b->get_shape().intersects(pair.a->get_shape())/*does_intersect(*pair.a,*pair.b)*/) {
                Body &aa = *pair.a;
                Body &bb = *pair.b;

                double e = std::min(aa.get_e(), bb.get_e());
                Vec a_pos = aa.get_position();
                Vec b_pos = bb.get_position();
                Vec collision_normal = b_pos - a_pos;
                collision_normal.normalize();
                Manifold m = Manifold{aa, bb, e, collision_normal};


                set_new_speeds(aa, bb, m);
                //double  ee = -1.0;
                pair.b->normal = Vec{m.normal.get_x() * -1, m.normal.get_y() * -1};
                pair.a->normal = m.normal;
                position_correction(aa, bb, m);

            }

        }for(Body* body : bodies){
            body->collides_wall(h,w,dt);
            body->integrate(dt,w,h);
            body->normal = Vec{0,0};
            body->set_current(body->get_position());


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



    void update(State& state){
        std::vector<Body*> state_bodies = state.get_bodies();
        bodies.clear();
        for (Body* body : state_bodies){

            Body& bod = *body;
            DrawBody draw_body{bod, state.get_h(), sf::Color::White};
            bodies.push_back(draw_body);
        }
    }
    std::vector<DrawBody> bodies;
};






int main() {
    //text stuff for velocities
    sf::Font font;
    font.loadFromFile("arial.ttf");
    sf::Text texta;
    sf::Text textb;
    texta.setFont(font);
    textb.setFont(font);
    texta.setCharacterSize(40); // in pixels, not points!
    textb.setCharacterSize(40); // in pixels, not points!
    texta.setFillColor(sf::Color(22, 23, 23));
    textb.setFillColor(sf::Color(22, 23, 23));

    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "Simple Physics Engine");
    double h = window.getSize().y;
    double w = window.getSize().x;
    Circle circ{100, Vec{w / 2, h / 2}};
    Circle circ2{20, Vec{900, 200}};
    Circle circ4{100, Vec{1500, 500}};
    Circle circ5{20, Vec{1000, 1200}};
    Circle circ3{50, Vec{(w/3)*2, (h/3)*2}};
    AABB ab{Vec{1,10}, Vec{w-1,100}};
    AABB ab3{Vec{w-100,200}, Vec{w-50,h-1}};
    Body boddd{ab3,0.75,0, Vec{0,0}};
    Body bod3{ab, 0.75, 0,Vec{0,0}};
   Body body2{circ4, 0.75, 0, Vec{0, 0}};
    Body body5{circ2, 0.75, 0.5, Vec{500, 800}};
    Body body6{circ5, 0.75, 0.5, Vec{500, 800}};
   Body body{circ, 0.75, 3, Vec{-200, 500}};
   AABB ab2{Vec{300,500}, Vec{600,780}};
   Body bod4{ab2, 0.75, 2.5,Vec{0,600}};
    std::vector<Body> bodes;
    std::vector<Circle> shapes;
   State state{w,h};
  state.add_body(&bod3);
   state.add_body(&body2);
  // state.add_body(&body);

  state.add_body(&bod4);
   //state.add_body(&boddd);
    state.add_body(&body5);
  //  state.add_body(&body6);
    std::vector<AABB> boxes;

    for(int i = 0; i < 1; i++){
        AABB box{Vec{10 + 20.0*i, h-100}, Vec{10 + 20*i + 10.0, h-10}};
        boxes.push_back(box);
    }

    std::vector<Body> bodies;
    for(AABB& box : boxes){
        Body the_bod{box,0.75,0.5,Vec{0,0}};
        bodies.push_back(the_bod);
    }

    for(Body& boder : bodies){
        state.add_body(&boder);
    }
 bool added = false;
    DrawBodies draw_bodies{};
    draw_bodies.update(state);

    Circle test{20, Vec{0,0}};
    Body tester{test,0.75,0.5,Vec{-200,0}};

    double fps = 60;
    double dt = 1 / fps;
    sf::Clock clock;
    double accum = 0;
    while (window.isOpen()) {

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }
        sf::Vector2i localPosition = sf::Mouse::getPosition(window);

        double x = 0;
        double y = 0;
        bool pressed = true;
        if(pressed && sf::Mouse::isButtonPressed(sf::Mouse::Left))
        {
            y = h -double (localPosition.y);
            x = double (localPosition.x);

        }

        if(x != 0 && y !=0 && !added){
           tester.set_position(x,y);
           state.add_body(&tester);
           draw_bodies.update(state);
           added = true;
        }
        for(Body bode : bodes){
            state.add_body(&bode);
        }
        draw_bodies.update(state);



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
        window.clear(sf::Color(22, 23, 23));
        std::string bod4_str = std::to_string(bod4.get_velocity().get_x()) + "\n" + std::to_string(bod4.get_velocity().get_y());
     //   std::string bod3_str = std::to_string(bod3.get_velocity().get_x()) + "\n" + std::to_string(bod3.get_velocity().get_y());
        texta.setString(bod4_str);
      //  textb.setString(bod3_str);
        texta.setPosition(bod4.get_shape().get_min().get_x(),h-bod4.get_position().get_y());
   //     textb.setPosition(bod3.get_shape().get_min().get_x(),h-bod3.get_position().get_y());
        //draw_bodies.draw_state(state,window);
         for(const DrawBody& bodd : draw_bodies.bodies){
            window.draw(bodd);
        }
         window.draw(texta);
      //  window.draw(textb);
        window.display();



    }









    return 0;
}